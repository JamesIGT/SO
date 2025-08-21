#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <pthread.h>

// ------------------------
// Configuración y constantes
// ------------------------

#define MAP_SIZE 100

// Zonas (eje Y)
#define ZONA_ENSAMBLE_MAX 33
#define ZONA_DEFENSA_MAX 66
#define ZONA_REENSAMBLE_MAX 100

#define NUM_ENJAMBRES 3
#define DRONES_POR_ENJAMBRE 5
#define TOTAL_DRONES (NUM_ENJAMBRES * DRONES_POR_ENJAMBRE)

// Tipos de comandos
enum {
    CMD_NONE = 0,
    CMD_TAKEOFF = 10,       // Orden de despegue
    CMD_PROCEED = 1,        // Ensamble -> Re-ensamble
    CMD_GO_ATTACK = 2,      // Re-ensamble -> Objetivo
    CMD_SET_SWARM = 3,      // Reasignación de enjambre (data = nuevo swarm)
    CMD_SET_TARGET = 4,     // data=x, data2=y
    CMD_SHUTDOWN = 99
};

// Tipos de eventos
enum {
    EVT_READY = 1,          // Llegó a punto de ensamble
    EVT_AT_REASSEMBLY = 2,  // Llegó a punto de re-ensamble
    EVT_AT_TARGET = 3,      // Llegó a objetivo
    EVT_DESTROYED = 4,      // Derribado por defensa
    EVT_DETONATED = 5,      // Bomba detonada
    EVT_FUEL_EMPTY = 6,     // Combustible agotado
    EVT_CAM_REPORT = 7,     // Informe cámara
    EVT_HEARTBEAT = 8,      // Latido de enlace
    EVT_LINK_LOST = 9,      // Enlace perdido
    EVT_LINK_RESTORED = 10, // Enlace restablecido
    EVT_DRONE_LOST = 11     // Dron dado por perdido
};

// Probabilidad de derribo por defensa (0..100) [por defecto]
#define DEFENSA_PROB_PCT 25
// Probabilidad de pérdida de enlace Q% (0..100) [por defecto]
#define LINK_LOSS_Q_PCT_DEFAULT 5
// Timeout Z para reconexión (segundos) [por defecto]
#define RECONNECT_TIMEOUT_SEC_DEFAULT 5

// Combustible por dron (ticks)
#define COMBUSTIBLE_INICIAL 600

// Velocidad por tick (en unidades del mapa)
#define VELOCIDAD_POR_TICK 1

// Tiempos (us)
#define TICK_USEC 50000

// ------------------------
// Estructuras
// ------------------------

struct Punto {
    int x;
    int y;
    char nombre[64];
};

struct CenterEvent {
    int type;
    int drone_id;
    int swarm_id;
    int data;
    char message[64];
};

struct DroneCommand {
    int cmd;
    int data;
    int data2;
};

// Estado compartido por todos los procesos (memoria compartida SysV)
struct DroneShared {
    int active;        // 1 si slot en uso
    int drone_id;      // único
    int swarm_id;      // 0..NUM_ENJAMBRES-1
    int is_camera;     // 1 si es dron de cámara
    int x;
    int y;
    int alive;         // 1 vivo, 0 destruido
    int ready;         // 1 si listo en ensamble
    int detonated;     // 1 si detonó
    int fuel;          // combustible restante
    int distance;      // distancia acumulada (unidades)
    int link_ok;       // estado de enlace simulado 1/0
};

// ------------------------
// Coordenadas fijas
// ------------------------

static struct Punto puntos_ensamblaje[NUM_ENJAMBRES] = {
    {25, 16, "Enjambre 1 - Punto de Ensamblaje"},
    {50, 16, "Enjambre 2 - Punto de Ensamblaje"},
    {75, 16, "Enjambre 3 - Punto de Ensamblaje"}
};

static struct Punto puntos_reensamblaje[NUM_ENJAMBRES] = {
    {25, 82, "Enjambre 1 - Punto de Reensamblaje"},
    {50, 82, "Enjambre 2 - Punto de Reensamblaje"},
    {75, 82, "Enjambre 3 - Punto de Reensamblaje"}
};

static struct Punto puntos_objetivo[NUM_ENJAMBRES] = {
    {25, 100, "Objetivo 1"},
    {50, 100, "Objetivo 2"},
    {75, 100, "Objetivo 3"}
};

// ------------------------
// Globals
// ------------------------

static const char *CENTER_FIFO_PATH = "/tmp/center_events.fifo";

// Memoria compartida
static int g_shmid = -1;
static struct DroneShared *g_shared = NULL;

// Utilidades
static inline int min_int(int a, int b) { return a < b ? a : b; }
static inline int max_int(int a, int b) { return a > b ? a : b; }
static inline int clamp_int(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Parámetros en tiempo de ejecución (config)
static int g_defensa_prob_pct = DEFENSA_PROB_PCT;
static int g_link_loss_q_pct = LINK_LOSS_Q_PCT_DEFAULT;
static int g_reconnect_timeout_sec = RECONNECT_TIMEOUT_SEC_DEFAULT;

static void cargar_configuracion(void) {
    const char *paths[2] = { "/workspace/drones.conf", "./drones.conf" };
    for (int p = 0; p < 2; p++) {
        FILE *f = fopen(paths[p], "r");
        if (!f) continue;
        char line[256];
        while (fgets(line, sizeof(line), f)) {
            char key[64];
            int val;
            if (sscanf(line, " %63[^=]=%d", key, &val) == 2) {
                if (strcmp(key, "W") == 0) g_defensa_prob_pct = clamp_int(val, 0, 100);
                else if (strcmp(key, "Q") == 0) g_link_loss_q_pct = clamp_int(val, 0, 100);
                else if (strcmp(key, "Z") == 0) g_reconnect_timeout_sec = clamp_int(val, 1, 60);
            }
        }
        fclose(f);
    }
}

static void sigint_handler(int sig) {
    (void)sig;
    if (g_shared != NULL) {
        shmdt(g_shared);
        g_shared = NULL;
    }
    if (g_shmid != -1) {
        // No removemos el segmento aquí deliberadamente para permitir a otros procesos desconectarse
    }
    unlink(CENTER_FIFO_PATH);
    // Los FIFOs por dron se intentarán eliminar al final por el centro
}

static void instalar_signal_handlers(void) {
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = sigint_handler;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
}

// ------------------------
// Funciones de ayuda
// ------------------------

static void ensure_fifo(const char *path) {
    struct stat st;
    if (stat(path, &st) == 0) {
        if (!S_ISFIFO(st.st_mode)) {
            unlink(path);
            if (mkfifo(path, 0666) != 0) {
                perror("mkfifo");
                exit(1);
            }
        }
    } else {
        if (mkfifo(path, 0666) != 0 && errno != EEXIST) {
            perror("mkfifo");
            exit(1);
        }
    }
}

static void fifo_path_drone_cmd(int drone_id, char *out, size_t out_len) {
    snprintf(out, out_len, "/tmp/drone_cmd_%d.fifo", drone_id);
}

static int open_center_fifo_reader(void) {
    ensure_fifo(CENTER_FIFO_PATH);
    int fd_r = open(CENTER_FIFO_PATH, O_RDONLY | O_NONBLOCK);
    if (fd_r < 0) {
        perror("open center fifo rdr");
        exit(1);
    }
    // Mantener extremo de escritura abierto para evitar EOF cuando no hay escritores
    int fd_w = open(CENTER_FIFO_PATH, O_WRONLY | O_NONBLOCK);
    if (fd_w < 0) {
        // No es fatal, pero ayuda a la robustez
    }
    return fd_r;
}

static int open_center_fifo_writer(void) {
    ensure_fifo(CENTER_FIFO_PATH);
    // Espera a que el lector esté listo
    int fd = -1;
    for (int i = 0; i < 50; i++) {
        fd = open(CENTER_FIFO_PATH, O_WRONLY | O_NONBLOCK);
        if (fd >= 0) break;
        usleep(100000);
    }
    if (fd < 0) {
        perror("open center fifo wr");
        exit(1);
    }
    return fd;
}

static void write_event_fd(int fd, int type, int drone_id, int swarm_id, int data, const char *msg) {
    struct CenterEvent evt;
    memset(&evt, 0, sizeof(evt));
    evt.type = type;
    evt.drone_id = drone_id;
    evt.swarm_id = swarm_id;
    evt.data = data;
    if (msg) {
        snprintf(evt.message, sizeof(evt.message), "%s", msg);
    }
    ssize_t w = write(fd, &evt, sizeof(evt));
    (void)w;
}

static void write_event(int type, int drone_id, int swarm_id, int data, const char *msg) {
    int fd = open_center_fifo_writer();
    write_event_fd(fd, type, drone_id, swarm_id, data, msg);
    close(fd);
}

static int create_or_attach_shm(int create) {
    key_t key = ftok("/tmp", 'D');
    if (key == -1) {
        perror("ftok");
        exit(1);
    }
    int flags = 0666 | (create ? IPC_CREAT : 0);
    int shmid = shmget(key, sizeof(struct DroneShared) * TOTAL_DRONES, flags);
    if (shmid < 0) {
        perror("shmget");
        exit(1);
    }
    void *mem = shmat(shmid, NULL, 0);
    if (mem == (void *)-1) {
        perror("shmat");
        exit(1);
    }
    g_shmid = shmid;
    g_shared = (struct DroneShared *)mem;
    return shmid;
}

static void init_shared(void) {
    for (int i = 0; i < TOTAL_DRONES; i++) {
        g_shared[i].active = 0;
        g_shared[i].drone_id = -1;
        g_shared[i].swarm_id = -1;
        g_shared[i].is_camera = 0;
        g_shared[i].x = 0;
        g_shared[i].y = 0;
        g_shared[i].alive = 0;
        g_shared[i].ready = 0;
        g_shared[i].detonated = 0;
        g_shared[i].fuel = 0;
        g_shared[i].distance = 0;
        g_shared[i].link_ok = 1;
    }
}

static int idx_for_drone(int drone_id) {
    for (int i = 0; i < TOTAL_DRONES; i++) {
        if (g_shared[i].active && g_shared[i].drone_id == drone_id) return i;
    }
    return -1;
}

// ------------------------
// Drones
// ------------------------

struct DroneCtx {
    int drone_id;
    int swarm_id;
    int is_camera;
    int cmd_fd;       // FIFO comandos (lectura)
    volatile int proceed_allowed;
    volatile int attack_allowed;
    volatile int takeoff_allowed;
    volatile int alive;
    volatile int at_target;
    int center_fd;    // FIFO eventos (escritura)
    volatile int link_ok;
    int target_x;
    int target_y;
};

static void *hilo_combustible(void *arg) {
    struct DroneCtx *ctx = (struct DroneCtx *)arg;
    int idx = idx_for_drone(ctx->drone_id);
    if (idx < 0) return NULL;
    int last_distance = 0;
    while (ctx->alive) {
        usleep(TICK_USEC);
        if (!ctx->alive) break;
        int current_distance = g_shared[idx].distance;
        int delta = max_int(0, current_distance - last_distance);
        last_distance = current_distance;
        int fuel = g_shared[idx].fuel - delta;
        fuel = max_int(0, fuel);
        g_shared[idx].fuel = fuel;
        if (fuel == 0) {
            ctx->alive = 0;
            g_shared[idx].alive = 0;
            write_event_fd(ctx->center_fd, EVT_FUEL_EMPTY, ctx->drone_id, g_shared[idx].swarm_id, 0, "Fuel 0");
            break;
        }
    }
    return NULL;
}

static void mover_hacia(struct DroneCtx *ctx, int dest_x, int dest_y) {
    int idx = idx_for_drone(ctx->drone_id);
    if (idx < 0) return;
    while (ctx->alive) {
        int dx = dest_x - g_shared[idx].x;
        int dy = dest_y - g_shared[idx].y;
        if (dx == 0 && dy == 0) break;
        int step_x = 0, step_y = 0;
        if (dx != 0) { step_x = (dx > 0 ? 1 : -1) * min_int(abs(dx), VELOCIDAD_POR_TICK); g_shared[idx].x += step_x; }
        if (dy != 0) { step_y = (dy > 0 ? 1 : -1) * min_int(abs(dy), VELOCIDAD_POR_TICK); g_shared[idx].y += step_y; }
        g_shared[idx].distance += abs(step_x) + abs(step_y);
        // Chequear si defensa lo destruyó
        if (!g_shared[idx].alive) {
            ctx->alive = 0;
            break;
        }
        usleep(TICK_USEC);
    }
}

static void loiter_en_circulos(struct DroneCtx *ctx, int cx, int cy, int radio, int vueltas_minimas) {
    int idx = idx_for_drone(ctx->drone_id);
    if (idx < 0) return;
    int pasos = 0;
    int perimetro = radio * 8; // aproximación cuadrada
    int fase = 0;
    while (ctx->alive && !ctx->proceed_allowed) {
        int x = g_shared[idx].x;
        int y = g_shared[idx].y;
        int tx = x, ty = y;
        switch ((fase / radio) % 4) {
            case 0: tx = min_int(cx + radio, x + 1); break;
            case 1: ty = min_int(cy + radio, y + 1); break;
            case 2: tx = max_int(cx - radio, x - 1); break;
            case 3: ty = max_int(cy - radio, y - 1); break;
        }
        mover_hacia(ctx, tx, ty);
        pasos++;
        fase++;
        if (vueltas_minimas > 0 && pasos > perimetro * vueltas_minimas) vueltas_minimas = 0;
        usleep(TICK_USEC);
    }
}

static void *hilo_comunicacion(void *arg) {
    struct DroneCtx *ctx = (struct DroneCtx *)arg;
    int link_down_since = 0;
    for (;;) {
        struct DroneCommand cmd;
        ssize_t r = read(ctx->cmd_fd, &cmd, sizeof(cmd));
        if (r == sizeof(cmd)) {
            if (!ctx->link_ok) {
                // Ignora comandos cuando no hay enlace
            } else if (cmd.cmd == CMD_TAKEOFF) {
                ctx->takeoff_allowed = 1;
            } else if (cmd.cmd == CMD_PROCEED) ctx->proceed_allowed = 1;
            else if (cmd.cmd == CMD_GO_ATTACK) ctx->attack_allowed = 1;
            else if (cmd.cmd == CMD_SET_SWARM) {
                int idx = idx_for_drone(ctx->drone_id);
                if (idx >= 0) {
                    g_shared[idx].swarm_id = cmd.data;
                }
            } else if (cmd.cmd == CMD_SET_TARGET) {
                ctx->target_x = cmd.data;
                ctx->target_y = cmd.data2;
            } else if (cmd.cmd == CMD_SHUTDOWN) {
                ctx->alive = 0;
                int idx = idx_for_drone(ctx->drone_id);
                if (idx >= 0) g_shared[idx].alive = 0;
                break;
            }
        } else {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                usleep(100000);
                continue;
            } else if (r == 0) {
                // Reabrir por si el escritor cerró
                close(ctx->cmd_fd);
                // Vuelve a abrir en no bloqueante
                char path[128];
                fifo_path_drone_cmd(ctx->drone_id, path, sizeof(path));
                ctx->cmd_fd = open(path, O_RDONLY | O_NONBLOCK);
                if (ctx->cmd_fd < 0) break;
            } else {
                usleep(100000);
            }
        }
        // Simular pérdida de enlace con probabilidad Q% por segundo
        static const int period_ticks = 20; // ~1s si TICK_USEC=50ms
        static __thread int tick_counter = 0;
        tick_counter++;
        if (tick_counter >= period_ticks) {
            tick_counter = 0;
            if (ctx->link_ok) {
                int r = rand() % 100;
                if (r < g_link_loss_q_pct) {
                    ctx->link_ok = 0;
                    int idx = idx_for_drone(ctx->drone_id);
                    if (idx >= 0) g_shared[idx].link_ok = 0;
                    write_event_fd(ctx->center_fd, EVT_LINK_LOST, ctx->drone_id, (idx>=0?g_shared[idx].swarm_id:-1), 0, "link down");
                    link_down_since = (int)time(NULL);
                }
            } else {
                int r2 = rand() % 100;
                if (r2 < 50) {
                    ctx->link_ok = 1;
                    int idx = idx_for_drone(ctx->drone_id);
                    if (idx >= 0) g_shared[idx].link_ok = 1;
                    write_event_fd(ctx->center_fd, EVT_LINK_RESTORED, ctx->drone_id, (idx>=0?g_shared[idx].swarm_id:-1), 0, "link up");
                } else {
                    if ((int)time(NULL) - link_down_since >= g_reconnect_timeout_sec) {
                        // Pérdida del drone
                        ctx->alive = 0;
                        int idx = idx_for_drone(ctx->drone_id);
                        if (idx >= 0) g_shared[idx].alive = 0;
                        write_event_fd(ctx->center_fd, EVT_DRONE_LOST, ctx->drone_id, (idx>=0?g_shared[idx].swarm_id:-1), 0, "lost");
                        break;
                    }
                }
            }
        }
        if (!ctx->alive) break;
    }
    return NULL;
}

static void *hilo_armas(void *arg) {
    struct DroneCtx *ctx = (struct DroneCtx *)arg;
    if (ctx->is_camera) return NULL;
    int idx = idx_for_drone(ctx->drone_id);
    if (idx < 0) return NULL;
    while (ctx->alive) {
        if (ctx->at_target) {
            // Detona
            write_event_fd(ctx->center_fd, EVT_DETONATED, ctx->drone_id, g_shared[idx].swarm_id, 0, "Boom");
            g_shared[idx].detonated = 1;
            g_shared[idx].alive = 0;
            ctx->alive = 0;
            break;
        }
        usleep(100000);
    }
    return NULL;
}

static void *hilo_camara(void *arg) {
    struct DroneCtx *ctx = (struct DroneCtx *)arg;
    if (!ctx->is_camera) return NULL;
    int idx = idx_for_drone(ctx->drone_id);
    if (idx < 0) return NULL;
    while (ctx->alive) {
        if (ctx->at_target) {
            // Espera a que los de ataque del mismo enjambre detonen o mueran
            int swarm_id = g_shared[idx].swarm_id;
            int detonados = 0;
            int vivos_ataque = 0;
            for (int i = 0; i < TOTAL_DRONES; i++) {
                if (!g_shared[i].active) continue;
                if (g_shared[i].swarm_id != swarm_id) continue;
                if (g_shared[i].is_camera) continue;
                if (g_shared[i].alive) vivos_ataque++;
                if (g_shared[i].detonated) detonados++;
            }
            if (detonados >= 2 || vivos_ataque == 0) {
                int resultado = (detonados >= 2) ? 1 : 0; // 1 destruido, 0 parcial
                write_event_fd(ctx->center_fd, EVT_CAM_REPORT, ctx->drone_id, swarm_id, resultado, resultado ? "Objetivo destruido" : "Objetivo parcial");
                // Autodestrucción
                g_shared[idx].alive = 0;
                ctx->alive = 0;
                break;
            }
        }
        usleep(200000);
    }
    return NULL;
}

static void *hilo_navegacion(void *arg) {
    struct DroneCtx *ctx = (struct DroneCtx *)arg;
    int idx = idx_for_drone(ctx->drone_id);
    if (idx < 0) return NULL;

    // Espera despegue
    while (ctx->alive && !ctx->takeoff_allowed) usleep(100000);

    // 1) Camión -> Ensamble
    struct Punto pe = puntos_ensamblaje[g_shared[idx].swarm_id];
    mover_hacia(ctx, pe.x, pe.y);
    if (ctx->alive) {
        g_shared[idx].ready = 1;
        write_event_fd(ctx->center_fd, EVT_READY, ctx->drone_id, g_shared[idx].swarm_id, 0, "READY ensamble");
    }

    // 2) Loiter en círculo hasta que el enjambre esté completo
    loiter_en_circulos(ctx, pe.x, pe.y, 2, 1);

    // 2b) Avanza hasta re-ensamble
    struct Punto pr = puntos_reensamblaje[g_shared[idx].swarm_id];
    mover_hacia(ctx, pr.x, pr.y);
    if (ctx->alive) {
        write_event_fd(ctx->center_fd, EVT_AT_REASSEMBLY, ctx->drone_id, g_shared[idx].swarm_id, 0, "En re-ensamble");
    }

    // 3) Espera CMD_GO_ATTACK y avanza al objetivo
    while (ctx->alive && !ctx->attack_allowed) usleep(100000);
    int tx = ctx->target_x;
    int ty = ctx->target_y;
    if (ty <= 0) { // fallback
        struct Punto po = puntos_objetivo[g_shared[idx].swarm_id];
        tx = po.x; ty = po.y;
    }
    mover_hacia(ctx, tx, ty);
    if (ctx->alive) {
        ctx->at_target = 1;
        write_event_fd(ctx->center_fd, EVT_AT_TARGET, ctx->drone_id, g_shared[idx].swarm_id, 0, "En objetivo");
    }

    return NULL;
}

static void crear_fifo_comandos_dron(int drone_id, int *fd_out) {
    char path[128];
    fifo_path_drone_cmd(drone_id, path, sizeof(path));
    ensure_fifo(path);
    int fd = open(path, O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        perror("open drone cmd fifo");
        exit(1);
    }
    *fd_out = fd;
}

static void enviar_comando(int drone_id, int cmd, int data, int data2) {
    char path[128];
    fifo_path_drone_cmd(drone_id, path, sizeof(path));
    int fd = open(path, O_WRONLY | O_NONBLOCK);
    if (fd < 0) return;
    struct DroneCommand c = {cmd, data, data2};
    write(fd, &c, sizeof(c));
    close(fd);
}

static void proceso_dron(int drone_id, int swarm_id_inicial, int es_camara) {
    // Semilla RNG
    srand((unsigned int)(time(NULL) ^ getpid()));

    // Adjuntar memoria compartida y registrar slot
    create_or_attach_shm(0);
    int slot = -1;
    for (int i = 0; i < TOTAL_DRONES; i++) {
        if (!g_shared[i].active) { slot = i; break; }
    }
    if (slot < 0) exit(1);
    g_shared[slot].active = 1;
    g_shared[slot].drone_id = drone_id;
    g_shared[slot].swarm_id = swarm_id_inicial;
    g_shared[slot].is_camera = es_camara;
    g_shared[slot].x = puntos_ensamblaje[swarm_id_inicial].x;
    g_shared[slot].y = 0; // parte desde camión (y=0)
    g_shared[slot].alive = 1;
    g_shared[slot].ready = 0;
    g_shared[slot].detonated = 0;
    g_shared[slot].fuel = COMBUSTIBLE_INICIAL;
    g_shared[slot].distance = 0;
    g_shared[slot].link_ok = 1;

    int cmd_fd = -1;
    crear_fifo_comandos_dron(drone_id, &cmd_fd);
    int center_fd = open_center_fifo_writer();

    struct DroneCtx ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.drone_id = drone_id;
    ctx.swarm_id = swarm_id_inicial;
    ctx.is_camera = es_camara;
    ctx.proceed_allowed = 0;
    ctx.attack_allowed = 0;
    ctx.takeoff_allowed = 0;
    ctx.alive = 1;
    ctx.at_target = 0;
    ctx.cmd_fd = cmd_fd;
    ctx.center_fd = center_fd;
    ctx.link_ok = 1;
    ctx.target_x = puntos_objetivo[swarm_id_inicial].x;
    ctx.target_y = puntos_objetivo[swarm_id_inicial].y;

    pthread_t th_nav, th_fuel, th_comm, th_role;
    pthread_create(&th_comm, NULL, hilo_comunicacion, &ctx);
    pthread_create(&th_fuel, NULL, hilo_combustible, &ctx);
    pthread_create(&th_nav, NULL, hilo_navegacion, &ctx);
    if (es_camara) pthread_create(&th_role, NULL, hilo_camara, &ctx);
    else pthread_create(&th_role, NULL, hilo_armas, &ctx);

    pthread_join(th_nav, NULL);
    ctx.alive = 0;
    pthread_join(th_role, NULL);
    pthread_join(th_fuel, NULL);
    pthread_cancel(th_comm);
    close(cmd_fd);
    close(center_fd);
    // Mantener datos finales
    _exit(0);
}

// ------------------------
// Enjambre (camión)
// ------------------------

static void enviar_comando_enjambre(int swarm_id, int cmd) {
    for (int i = 0; i < TOTAL_DRONES; i++) {
        if (!g_shared[i].active) continue;
        if (g_shared[i].swarm_id == swarm_id) {
            enviar_comando(g_shared[i].drone_id, cmd, 0, 0);
        }
    }
}

static void crear_enjambre(int camion_id, struct Punto punto_ensamble, struct Punto punto_reensamble, struct Punto objetivo) {
    (void)camion_id; (void)punto_ensamble; (void)punto_reensamblaje; (void)objetivo;
    // Crea 5 procesos dron: 4 ataque (idx 0..3), 1 cámara (idx 4)
    int swarm_id = camion_id; // 0..NUM_ENJAMBRES-1
    for (int i = 0; i < DRONES_POR_ENJAMBRE; i++) {
        pid_t pid = fork();
        if (pid == 0) {
            int drone_id = swarm_id * 100 + i; // único simple
            int es_camara = (i == DRONES_POR_ENJAMBRE - 1) ? 1 : 0;
            proceso_dron(drone_id, swarm_id, es_camara);
            exit(0);
        }
    }
}

static void proceso_enjambre(int camion_id) {
    // Crea sus drones
    crear_enjambre(camion_id, puntos_ensamblaje[camion_id], puntos_reensamblaje[camion_id], puntos_objetivo[camion_id]);
    // En este modelo, el camión no reenvía eventos; el Centro escucha directamente
    // Mantener el proceso vivo hasta que el centro termine
    for (;;) {
        sleep(1);
    }
}

// ------------------------
// Defensas anti-drone
// ------------------------

static void defensa_main(int def_x, int def_y, int prob_pct) {
    srand((unsigned int)(time(NULL) ^ getpid()));
    create_or_attach_shm(0);
    int center_fd = open_center_fifo_writer();
    (void)def_y; // No usado directamente; el criterio es y en [33,66]
    for (;;) {
        for (int i = 0; i < TOTAL_DRONES; i++) {
            if (!g_shared[i].active) continue;
            if (!g_shared[i].alive) continue;
            int y = g_shared[i].y;
            if (y >= ZONA_ENSAMBLE_MAX && y <= ZONA_DEFENSA_MAX) {
                int x = g_shared[i].x;
                if (abs(x - def_x) <= 15) {
                    int r = rand() % 100;
                    if (r < prob_pct) {
                        g_shared[i].alive = 0;
                        write_event_fd(center_fd, EVT_DESTROYED, g_shared[i].drone_id, g_shared[i].swarm_id, 0, "Derribado");
                    }
                }
            }
        }
        usleep(200000);
    }
}

// ------------------------
// Centro de comando
// ------------------------

static void difundir_a_todos(int cmd) {
    for (int i = 0; i < TOTAL_DRONES; i++) {
        if (!g_shared[i].active) continue;
        enviar_comando(g_shared[i].drone_id, cmd, 0, 0);
    }
}

// Retasking: alternado izquierda/derecha desde cada enjambre incompleto, exclusivamente
static void retask_reensamble(void) {
    int conteo[NUM_ENJAMBRES] = {0};
    int locked[TOTAL_DRONES] = {0};
    for (int i = 0; i < TOTAL_DRONES; i++) {
        if (!g_shared[i].active) continue;
        if (!g_shared[i].alive) continue;
        if (g_shared[i].y >= ZONA_DEFENSA_MAX) conteo[g_shared[i].swarm_id]++;
    }
    // Para cada swarm incompleto, buscar donantes alternando offsets 1, -1, 2, -2, ...
    for (int s_need = 0; s_need < NUM_ENJAMBRES; s_need++) {
        while (conteo[s_need] < DRONES_POR_ENJAMBRE) {
            int found = 0;
            for (int off = 1; off < NUM_ENJAMBRES && conteo[s_need] < DRONES_POR_ENJAMBRE; off++) {
                int cand[2] = { s_need - off, s_need + off };
                for (int c = 0; c < 2; c++) {
                    int s_give = cand[c];
                    if (s_give < 0 || s_give >= NUM_ENJAMBRES) continue;
                    if (conteo[s_give] <= DRONES_POR_ENJAMBRE) continue; // no extraer de completos o deficitarios
                    // elegir un dron donante no cámara y no bloqueado
                    int donor = -1;
                    for (int i = 0; i < TOTAL_DRONES; i++) {
                        if (!g_shared[i].active) continue;
                        if (!g_shared[i].alive) continue;
                        if (g_shared[i].swarm_id != s_give) continue;
                        if (g_shared[i].y < ZONA_DEFENSA_MAX) continue; // sólo los que llegaron a re-ensamble
                        if (g_shared[i].is_camera) continue; // preferir no mover cámara
                        if (locked[i]) continue;
                        donor = i; break;
                    }
                    if (donor < 0) {
                        for (int i = 0; i < TOTAL_DRONES; i++) {
                            if (!g_shared[i].active) continue;
                            if (!g_shared[i].alive) continue;
                            if (g_shared[i].swarm_id != s_give) continue;
                            if (g_shared[i].y < ZONA_DEFENSA_MAX) continue;
                            if (locked[i]) continue;
                            donor = i; break; // como fallback, incluso cámara
                        }
                    }
                    if (donor >= 0) {
                        int drone_id = g_shared[donor].drone_id;
                        locked[donor] = 1; // exclusividad
                        enviar_comando(drone_id, CMD_SET_SWARM, s_need, 0);
                        g_shared[donor].swarm_id = s_need;
                        conteo[s_give]--;
                        conteo[s_need]++;
                        found = 1;
                        if (conteo[s_need] >= DRONES_POR_ENJAMBRE) break;
                    }
                }
                if (found && conteo[s_need] >= DRONES_POR_ENJAMBRE) break;
            }
            if (!found) break; // no más donantes
        }
    }
}

static void informe_final(int swarm_id, int estado) {
    const char *s = estado ? "DESTRUIDO" : "PARCIAL/INCOMPLETO";
    fprintf(stderr, "[Centro] Resultado Enjambre %d: %s\n", swarm_id + 1, s);
}

static void centro_de_comando(void) {
    instalar_signal_handlers();
    create_or_attach_shm(1);
    init_shared();
    int fd_evt = open_center_fifo_reader();
    cargar_configuracion();

    // Lanzar defensas
    pid_t d1 = fork();
    if (d1 == 0) defensa_main(10, 100, g_defensa_prob_pct);
    pid_t d2 = fork();
    if (d2 == 0) defensa_main(90, 100, g_defensa_prob_pct);

    // Lanzar enjambres (camiones)
    for (int s = 0; s < NUM_ENJAMBRES; s++) {
        pid_t p = fork();
        if (p == 0) {
            proceso_enjambre(s);
            exit(0);
        }
    }

    int ready_count[NUM_ENJAMBRES] = {0};
    int have_sent_proceed = 0;
    int reassembly_arrived[NUM_ENJAMBRES] = {0};
    int have_sent_attack = 0;
    int cam_reports[NUM_ENJAMBRES] = {0};

    // Asignación aleatoria de objetivos por ola
    int target_x[NUM_ENJAMBRES];
    int target_y[NUM_ENJAMBRES];
    for (int s = 0; s < NUM_ENJAMBRES; s++) {
        target_x[s] = puntos_objetivo[s].x;
        target_y[s] = puntos_objetivo[s].y;
    }

    // Enviar TAKEOFF a todos al inicio
    fprintf(stderr, "[Centro] CMD_TAKEOFF\n");
    for (int i = 0; i < TOTAL_DRONES; i++) if (g_shared[i].active) enviar_comando(g_shared[i].drone_id, CMD_TAKEOFF, 0, 0);

    time_t start = time(NULL);

    // Hilo de display de estado
    pthread_t th_disp;
    void *display_fn(void *arg) {
        (void)arg;
        for (;;) {
            usleep(500000);
            fprintf(stderr, "\n===== ESTADO =====\n");
            for (int s = 0; s < NUM_ENJAMBRES; s++) {
                int vivos = 0, listos = 0, en_re = 0, deton = 0;
                for (int i = 0; i < TOTAL_DRONES; i++) {
                    if (!g_shared[i].active) continue;
                    if (g_shared[i].swarm_id != s) continue;
                    if (g_shared[i].alive) vivos++;
                    if (g_shared[i].ready) listos++;
                    if (g_shared[i].y >= ZONA_DEFENSA_MAX) en_re++;
                    if (g_shared[i].detonated) deton++;
                }
                fprintf(stderr, "Enjambre %d: vivos=%d listos=%d re-ens=%d det=%d\n", s+1, vivos, listos, en_re, deton);
            }
        }
        return NULL;
    }
    pthread_create(&th_disp, NULL, display_fn, NULL);

    for (;;) {
        struct CenterEvent evt;
        ssize_t r = read(fd_evt, &evt, sizeof(evt));
        if (r == sizeof(evt)) {
            if (evt.type == EVT_READY) {
                if (evt.swarm_id >= 0 && evt.swarm_id < NUM_ENJAMBRES) {
                    ready_count[evt.swarm_id]++;
                    fprintf(stderr, "[Centro] READY dron %d (enjambre %d) -> %d/%d\n", evt.drone_id, evt.swarm_id + 1, ready_count[evt.swarm_id], DRONES_POR_ENJAMBRE);
                }
            } else if (evt.type == EVT_AT_REASSEMBLY) {
                if (evt.swarm_id >= 0 && evt.swarm_id < NUM_ENJAMBRES) {
                    reassembly_arrived[evt.swarm_id]++;
                    fprintf(stderr, "[Centro] Re-ensamble dron %d (enjambre %d)\n", evt.drone_id, evt.swarm_id + 1);
                }
            } else if (evt.type == EVT_DESTROYED) {
                fprintf(stderr, "[Centro] Dron %d derribado (enjambre %d)\n", evt.drone_id, evt.swarm_id + 1);
            } else if (evt.type == EVT_FUEL_EMPTY) {
                fprintf(stderr, "[Centro] Dron %d sin combustible (enjambre %d)\n", evt.drone_id, evt.swarm_id + 1);
            } else if (evt.type == EVT_AT_TARGET) {
                fprintf(stderr, "[Centro] Dron %d llegó a objetivo (enjambre %d)\n", evt.drone_id, evt.swarm_id + 1);
            } else if (evt.type == EVT_DETONATED) {
                fprintf(stderr, "[Centro] Dron %d detonó (enjambre %d)\n", evt.drone_id, evt.swarm_id + 1);
            } else if (evt.type == EVT_CAM_REPORT) {
                fprintf(stderr, "[Centro] Cámara %d reporte: %s (enjambre %d)\n", evt.drone_id, evt.data ? "DESTRUIDO" : "PARCIAL", evt.swarm_id + 1);
                cam_reports[evt.swarm_id] = evt.data ? 1 : 0;
            } else if (evt.type == EVT_LINK_LOST) {
                fprintf(stderr, "[Centro] Enlace PERDIDO con dron %d (enjambre %d)\n", evt.drone_id, evt.swarm_id + 1);
            } else if (evt.type == EVT_LINK_RESTORED) {
                fprintf(stderr, "[Centro] Enlace RESTAURADO con dron %d (enjambre %d)\n", evt.drone_id, evt.swarm_id + 1);
            } else if (evt.type == EVT_DRONE_LOST) {
                fprintf(stderr, "[Centro] Dron %d dado por PERDIDO (enjambre %d)\n", evt.drone_id, evt.swarm_id + 1);
            }
        } else {
            usleep(50000);
        }

        // Una vez todos listos en ensamble, arrancar misión hacia re-ensamble
        if (!have_sent_proceed) {
            int all_ready = 1;
            for (int s = 0; s < NUM_ENJAMBRES; s++) {
                if (ready_count[s] < DRONES_POR_ENJAMBRE) { all_ready = 0; break; }
            }
            if (all_ready) {
                fprintf(stderr, "[Centro] Todos listos. CMD_PROCEED\n");
                difundir_a_todos(CMD_PROCEED);
                have_sent_proceed = 1;
            }
        }

        // Cuando suficientes han llegado a re-ensamble (o ha pasado tiempo), re-balancear y atacar
        if (have_sent_proceed && !have_sent_attack) {
            int arrived = 0;
            int alive_total = 0;
            for (int i = 0; i < TOTAL_DRONES; i++) if (g_shared[i].active && g_shared[i].alive) alive_total++;
            for (int s = 0; s < NUM_ENJAMBRES; s++) arrived += reassembly_arrived[s];
            if (arrived >= max_int(1, alive_total / 2) || time(NULL) - start > 10) {
                // Reasignación de enjambres incompletos
                retask_reensamble();
                // Objetivos aleatorios por enjambre (barajar posiciones X)
                int order[NUM_ENJAMBRES];
                for (int s = 0; s < NUM_ENJAMBRES; s++) order[s] = s;
                for (int i = NUM_ENJAMBRES - 1; i > 0; i--) {
                    int j = rand() % (i + 1);
                    int tmp = order[i]; order[i] = order[j]; order[j] = tmp;
                }
                for (int s = 0; s < NUM_ENJAMBRES; s++) {
                    int tgt = order[s];
                    target_x[s] = puntos_objetivo[tgt].x;
                    target_y[s] = puntos_objetivo[tgt].y;
                }
                // Enviar objetivos a drones por enjambre y GO_ATTACK
                for (int i = 0; i < TOTAL_DRONES; i++) {
                    if (!g_shared[i].active) continue;
                    int s = g_shared[i].swarm_id;
                    enviar_comando(g_shared[i].drone_id, CMD_SET_TARGET, target_x[s], target_y[s]);
                }
                fprintf(stderr, "[Centro] CMD_GO_ATTACK\n");
                difundir_a_todos(CMD_GO_ATTACK);
                have_sent_attack = 1;
            }
        }

        // Terminar cuando todas cámaras reporten o timeout
        int reported = 0;
        for (int s = 0; s < NUM_ENJAMBRES; s++) reported += (cam_reports[s] >= 0);
        int all_cams_done = 1;
        for (int s = 0; s < NUM_ENJAMBRES; s++) {
            int cam_id = s * 100 + (DRONES_POR_ENJAMBRE - 1);
            int idx = idx_for_drone(cam_id);
            if (idx >= 0 && g_shared[idx].alive) { all_cams_done = 0; break; }
        }
        if (have_sent_attack && all_cams_done) {
            for (int s = 0; s < NUM_ENJAMBRES; s++) informe_final(s, cam_reports[s]);
            break;
        }
    }

    // Apagar drones
    difundir_a_todos(CMD_SHUTDOWN);
    pthread_cancel(th_disp);
    // Limpieza de FIFOs por dron
    for (int s = 0; s < NUM_ENJAMBRES; s++) {
        for (int i = 0; i < DRONES_POR_ENJAMBRE; i++) {
            int drone_id = s * 100 + i;
            char path[128];
            fifo_path_drone_cmd(drone_id, path, sizeof(path));
            unlink(path);
        }
    }
    unlink(CENTER_FIFO_PATH);
}

int main(void) {
    // El proceso principal actúa como Centro de Comando
    centro_de_comando();
    return 0;
}
