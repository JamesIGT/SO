#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <time.h>
#include <math.h>
#include <signal.h>
#include <errno.h>
#include <stdarg.h>

// Constantes del sistema
#define MAP_WIDTH 100
#define MAP_HEIGHT 100
#define ASSEMBLY_ZONE_END 33
#define DEFENSE_ZONE_START 33
#define DEFENSE_ZONE_END 66
#define REASSEMBLY_ZONE_START 66
#define DRONES_PER_SWARM 5
#define ATTACK_DRONES_PER_SWARM 4
#define CAMERA_DRONES_PER_SWARM 1
#define NUM_TRUCKS 3
#define NUM_TARGETS 3
#define NUM_ASSEMBLY_POINTS 3
#define NUM_REASSEMBLY_POINTS 3
#define NUM_ENEMY_DEFENSES 2
#define MAX_DRONES 100
#define MAX_EVENTS 1000
#define FIFO_PATH "/tmp/drone_wars2"
#define MAX_MSG_SIZE 256

// Tipos de drone
typedef enum {
    DRONE_TYPE_ATTACK = 0,
    DRONE_TYPE_CAMERA = 1
} DroneType;

// Estados del drone
typedef enum {
    DRONE_STATE_CREATED = 0,
    DRONE_STATE_FLYING_TO_ASSEMBLY,
    DRONE_STATE_CIRCLING_ASSEMBLY,  // Nuevo estado: volando en círculos en zona de ensamble
    DRONE_STATE_READY,
    DRONE_STATE_REASSEMBLED,  // Nuevo estado: después del re-ensamblaje
    DRONE_STATE_FLYING_TO_TARGET,
    DRONE_STATE_AT_TARGET,
    DRONE_STATE_DETONATED,
    DRONE_STATE_DESTROYED,
    DRONE_STATE_MISSION_COMPLETE,  // Para drones cámara que completaron su misión
    DRONE_STATE_FUEL_EMPTY
} DroneState;

// Estados del objetivo
typedef enum {
    TARGET_STATE_INTACT = 0,
    TARGET_STATE_PARTIAL = 1,
    TARGET_STATE_DESTROYED = 2
} TargetState;

// Eventos del sistema
typedef enum {
    EVT_READY,
    EVT_AT_TARGET,
    EVT_DETONATED,
    EVT_CAM_REPORT_OK,
    EVT_CAM_REPORT_FAIL,
    EVT_DESTROYED,
    EVT_FUEL_EMPTY
} EventType;

// Comandos del sistema
typedef enum {
    CMD_GO_ATTACK_GLOBAL,
    CMD_RETASK,
    CMD_REPORT_OK,
    CMD_REPORT_FAIL
} CommandType;

// Estructura de posición
typedef struct {
    int x, y;
} Position;

// Estructura de drone
typedef struct {
    int id;
    int truck_id;
    int swarm_id;
    DroneType type;
    DroneState state;
    Position pos;
    Position target;
    int fuel;
    int max_fuel;
    int distance_traveled;
    int shoot_down_probability; // Probabilidad individual de derribo
    pthread_mutex_t mutex;
    pthread_cond_t condition;
    char fifo_name[64];
    int fifo_fd;
    pthread_t nav_thread;
    pthread_t fuel_thread;
    pthread_t comm_thread;
    pthread_t payload_thread;
    int active;
} Drone;

// Estructura de enjambre
typedef struct {
    int id;
    int truck_id;
    Drone* drones[DRONES_PER_SWARM];
    int ready_count;
    int active_count;
    Position assembly_point;
    Position reassembly_point;
    pthread_mutex_t mutex;
} Swarm;

// Estructura de camión
typedef struct {
    int id;
    Position pos;
    Swarm* swarm;
    int drone_count;
    int active;
} Truck;

// Estructura de objetivo
typedef struct {
    int id;
    Position pos;
    TargetState state;
    int attack_count;
    int required_attacks;
} Target;

// Estructura de defensa enemiga
typedef struct {
    int id;
    Position pos;
    int shoot_down_probability;
} EnemyDefense;

// Estructura de evento
typedef struct {
    EventType type;
    int drone_id;
    int swarm_id;
    int truck_id;
    char data[128];
    time_t timestamp;
} Event;

// Estructura de comando
typedef struct {
    CommandType type;
    int target_id;
    char data[128];
} Command;

// Variables globales del sistema
typedef struct {
    // Configuración
    int W; // Probabilidad de derribo por defensas
    int Q; // Probabilidad de pérdida de comunicación
    int Z; // Timeout de comunicación en segundos
    int speed; // Velocidad de movimiento
    int initial_fuel; // Combustible inicial
    int ticks; // Número de ticks de simulación
    
    // Componentes del sistema
    Truck trucks[NUM_TRUCKS];
    Target targets[NUM_TARGETS];
    EnemyDefense defenses[NUM_ENEMY_DEFENSES];
    Position assembly_points[NUM_ASSEMBLY_POINTS];
    Position reassembly_points[NUM_REASSEMBLY_POINTS];
    
    // Estado del sistema
    Swarm* swarms[MAX_DRONES / DRONES_PER_SWARM];
    int swarm_count;
    Drone* all_drones[MAX_DRONES];
    int drone_count;
    Event event_queue[MAX_EVENTS];
    int event_count;
    int event_head;
    int event_tail;
    
    // Sincronización
    pthread_mutex_t system_mutex;
    pthread_cond_t system_condition;
    int global_attack_commanded;
    int all_swarms_ready;
    
    // Archivos FIFO
    int center_fifo_fd;
    char center_fifo_name[64];
    
    // Control
    int simulation_running;
    int phase;
    
    // Asignación aleatoria de objetivos a enjambres
    int target_assignments[NUM_TRUCKS];
} SystemState;

// Variables globales
SystemState system_state;
pthread_mutex_t log_mutex;

// Declaraciones de función
void optimize_drone_distribution();
void fly_in_circles(Drone* drone);
int try_extract_drones_from_swarm(Swarm* target_swarm, int source_swarm_id, int* needed_attack, int* needed_camera, int target_swarm_id);
void wait_for_defense_zone_crossing();
void wait_for_reassembly_ready();
void command_final_attack();
void command_detonation();
void wait_for_all_drones_at_target();

// Funciones de utilidad
void log_message(const char* format, ...) {
    va_list args;
    va_start(args, format);
    
    pthread_mutex_lock(&log_mutex);
    time_t now = time(NULL);
    struct tm* tm_info = localtime(&now);
    char time_str[26];
    strftime(time_str, 26, "%Y-%m-%d %H:%M:%S", tm_info);
    
    printf("[%s] ", time_str);
    vprintf(format, args);
    printf("\n");
    fflush(stdout);
    
    pthread_mutex_unlock(&log_mutex);
    va_end(args);
}

// Función para calcular distancia entre dos posiciones
double calculate_distance(Position p1, Position p2) {
    int dx = p1.x - p2.x;
    int dy = p1.y - p2.y;
    return sqrt(dx * dx + dy * dy);
}

// Función para mover drone hacia un objetivo
void move_drone_towards(Drone* drone, Position target) {
    double distance = calculate_distance(drone->pos, target);
    if (distance <= system_state.speed) {
        drone->pos = target;
        drone->distance_traveled += (int)distance;
    } else {
        double ratio = system_state.speed / distance;
        int dx = (int)((target.x - drone->pos.x) * ratio);
        int dy = (int)((target.y - drone->pos.y) * ratio);
        drone->pos.x += dx;
        drone->pos.y += dy;
        drone->distance_traveled += system_state.speed;
    }
}

// Función para verificar si un drone está en una zona
int is_drone_in_zone(Drone* drone, int zone_start, int zone_end) {
    return drone->pos.y >= zone_start && drone->pos.y <= zone_end;
}

// Función para verificar probabilidad
int check_probability(int percentage) {
    // Función simple y precisa de probabilidad
    // percentage debe estar entre 0 y 100
    if (percentage <= 0) return 0;
    if (percentage >= 100) return 1;
    
    // Generar número aleatorio entre 0 y 99
    int random_value = rand() % 100;
    
    // Retornar true si el valor aleatorio es menor que el porcentaje
    return random_value < percentage;
}

// Función para crear nombre de FIFO
void create_fifo_name(char* buffer, int drone_id) {
    snprintf(buffer, 64, "%s/drone_%d", FIFO_PATH, drone_id);
}

// Función para crear FIFO
int create_drone_fifo(int drone_id) {
    char fifo_name[64];
    create_fifo_name(fifo_name, drone_id);
    
    if (mkfifo(fifo_name, 0666) == -1 && errno != EEXIST) {
        log_message("Error creando FIFO para drone %d: %s", drone_id, strerror(errno));
        return -1;
    }
    
    int fd = open(fifo_name, O_RDWR | O_NONBLOCK);
    if (fd == -1) {
        log_message("Error abriendo FIFO para drone %d: %s", drone_id, strerror(errno));
        return -1;
    }
    
    return fd;
}

// Función para enviar evento
void send_event(EventType type, int drone_id, int swarm_id, int truck_id, const char* data) {
    pthread_mutex_lock(&system_state.system_mutex);
    
    if (system_state.event_count < MAX_EVENTS) {
        Event* event = &system_state.event_queue[system_state.event_tail];
        event->type = type;
        event->drone_id = drone_id;
        event->swarm_id = swarm_id;
        event->truck_id = truck_id;
        event->timestamp = time(NULL);
        if (data) {
            strncpy(event->data, data, sizeof(event->data) - 1);
            event->data[sizeof(event->data) - 1] = '\0';
        } else {
            event->data[0] = '\0';
        }
        
        system_state.event_tail = (system_state.event_tail + 1) % MAX_EVENTS;
        system_state.event_count++;
        
        log_message("Evento enviado: %d (Drone %d, Swarm %d, Truck %d)", 
                   type, drone_id, swarm_id, truck_id);
    }
    
    pthread_mutex_unlock(&system_state.system_mutex);
}

// Función para enviar comando
void send_command(CommandType type, int target_id, const char* data) {
    Command cmd;
    cmd.type = type;
    cmd.target_id = target_id;
    if (data) {
        strncpy(cmd.data, data, sizeof(cmd.data) - 1);
        cmd.data[sizeof(cmd.data) - 1] = '\0';
    } else {
        cmd.data[0] = '\0';
    }
    
    if (write(system_state.center_fifo_fd, &cmd, sizeof(cmd)) == -1) {
        log_message("Error enviando comando: %s", strerror(errno));
    }
}

// Hilo de navegación del drone
void* drone_navigation_thread(void* arg) {
    Drone* drone = (Drone*)arg;
    
    while (drone->active && drone->state != DRONE_STATE_DESTROYED && system_state.simulation_running) {
        pthread_mutex_lock(&drone->mutex);
        
        switch (drone->state) {
            case DRONE_STATE_FLYING_TO_ASSEMBLY:
                if (calculate_distance(drone->pos, drone->target) <= system_state.speed) {
                    drone->pos = drone->target;
                    drone->state = DRONE_STATE_CIRCLING_ASSEMBLY;
                    log_message("Drone %d llegó al punto de ensamble, comenzando patrulla circular", drone->id);
                } else {
                    move_drone_towards(drone, drone->target);
                }
                break;
                
            case DRONE_STATE_CIRCLING_ASSEMBLY:
                // Volar en círculos alrededor del punto de ensamble
                fly_in_circles(drone);
                break;
                
            case DRONE_STATE_FLYING_TO_TARGET:
                if (calculate_distance(drone->pos, drone->target) <= system_state.speed) {
                    drone->pos = drone->target;
                    drone->state = DRONE_STATE_AT_TARGET;
                    log_message("Drone %d llegó al objetivo (distancia recorrida: %d unidades)", drone->id, drone->distance_traveled);
                    send_event(EVT_AT_TARGET, drone->id, drone->swarm_id, drone->truck_id, "AT_TARGET");
                } else {
                    move_drone_towards(drone, drone->target);
                    
                    // Verificar si está en zona de defensa (solo entre Y=33 y Y=66)
                    // Verificar solo ocasionalmente para reducir probabilidad acumulada
                    static int defense_check_counter = 0;
                    defense_check_counter++;
                    
                    if (is_drone_in_zone(drone, DEFENSE_ZONE_START, DEFENSE_ZONE_END) && 
                        (defense_check_counter % 5 == 0)) { // Solo verificar cada 5 ticks (500ms)
                        if (check_probability(drone->shoot_down_probability)) {
                            drone->state = DRONE_STATE_DESTROYED;
                            log_message("Drone %d derribado por defensas enemigas en zona de defensa (Y=%d)", 
                                       drone->id, drone->pos.y);
                            send_event(EVT_DESTROYED, drone->id, drone->swarm_id, drone->truck_id, "SHOT_DOWN");
                            break;
                        }
                    }
                }
                break;
        }
        
        pthread_mutex_unlock(&drone->mutex);
        usleep(100000); // 100ms
    }
    
    return NULL;
}

// Hilo de combustible del drone
void* drone_fuel_thread(void* arg) {
    Drone* drone = (Drone*)arg;
    
    while (drone->active && drone->state != DRONE_STATE_DESTROYED && system_state.simulation_running) {
        pthread_mutex_lock(&drone->mutex);
        
        drone->fuel--;
        if (drone->fuel <= 0) {
            drone->state = DRONE_STATE_FUEL_EMPTY;
            log_message("Drone %d se quedó sin combustible", drone->id);
            send_event(EVT_FUEL_EMPTY, drone->id, drone->swarm_id, drone->truck_id, "FUEL_EMPTY");
            break;
        }
        
        pthread_mutex_unlock(&drone->mutex);
        sleep(1); // 1 segundo por tick
    }
    
    return NULL;
}

// Hilo de comunicación del drone
void* drone_communication_thread(void* arg) {
    Drone* drone = (Drone*)arg;
    
    while (drone->active && drone->state != DRONE_STATE_DESTROYED && system_state.simulation_running) {
        // Los drones nunca pierden la comunicación
        // Solo mantener el hilo activo para sincronización
        usleep(100000); // 100ms
    }
    
    return NULL;
}

// Hilo de payload del drone
void* drone_payload_thread(void* arg) {
    Drone* drone = (Drone*)arg;
    
    while (drone->active && drone->state != DRONE_STATE_DESTROYED && system_state.simulation_running) {
        pthread_mutex_lock(&drone->mutex);
        
        if (drone->state == DRONE_STATE_AT_TARGET) {
            if (drone->type == DRONE_TYPE_ATTACK) {
                // Drone de ataque espera comando para detonar
                // NO detona automáticamente
                static int logged_attack_drones[100] = {0}; // Array para evitar mensajes repetidos
                if (!logged_attack_drones[drone->id]) {
                    log_message("Drone de ataque %d llegó al objetivo, esperando comando para detonar", drone->id);
                    logged_attack_drones[drone->id] = 1;
                }
                
            } else if (drone->type == DRONE_TYPE_CAMERA) {
                // Drone cámara NO hace reporte automático aquí
                // Solo espera a que los drones de ataque detonen
                static int logged_camera_drones[100] = {0}; // Array para evitar mensajes repetidos
                if (!logged_camera_drones[drone->id]) {
                    log_message("Drone cámara %d en posición de vigilancia, esperando detonaciones", drone->id);
                    logged_camera_drones[drone->id] = 1;
                }
            }
        }
        
        pthread_mutex_unlock(&drone->mutex);
        usleep(100000); // 100ms
    }
    
    return NULL;
}

// Función para volar en círculos alrededor del punto de ensamble
void fly_in_circles(Drone* drone) {
    // Radio del círculo de patrulla
    const int patrol_radius = 3;
    
    // Calcular el centro del círculo (punto de ensamble)
    Position center = drone->target;
    
    // Calcular ángulo actual basado en el tiempo
    static int angle_counter = 0;
    angle_counter = (angle_counter + 1) % 360;
    
    // Convertir ángulo a radianes
    double angle_rad = (angle_counter * 3.14159) / 180.0;
    
    // Calcular nueva posición en el círculo
    int new_x = center.x + (int)(patrol_radius * cos(angle_rad));
    int new_y = center.y + (int)(patrol_radius * sin(angle_rad));
    
    // Mover drone a la nueva posición
    drone->pos.x = new_x;
    drone->pos.y = new_y;
    
    // Incrementar distancia recorrida
    drone->distance_traveled += system_state.speed;
    
    // Verificar si el enjambre está listo para avanzar
    // (esto se maneja en el centro de comando)
}

// Función para crear un drone
Drone* create_drone(int id, int truck_id, int swarm_id, DroneType type, Position start_pos, Position target_pos) {
    Drone* drone = malloc(sizeof(Drone));
    if (!drone) {
        log_message("Error: No se pudo asignar memoria para drone %d", id);
        return NULL;
    }
    
    drone->id = id;
    drone->truck_id = truck_id;
    drone->swarm_id = swarm_id;
    drone->type = type;
    drone->state = DRONE_STATE_FLYING_TO_ASSEMBLY;
    drone->pos = start_pos;
    drone->target = target_pos;
    drone->fuel = system_state.initial_fuel;
    drone->max_fuel = system_state.initial_fuel;
    drone->distance_traveled = 0;
    
    // Asignar probabilidad individual de derribo (0% a 5%)
    drone->shoot_down_probability = rand() % 6; // 0 a 5
    
    drone->active = 1;
    
    // Inicializar mutex y condition variable
    pthread_mutex_init(&drone->mutex, NULL);
    pthread_cond_init(&drone->condition, NULL);
    
    // Crear FIFO para comunicación
    create_fifo_name(drone->fifo_name, id);
    drone->fifo_fd = create_drone_fifo(id);
    
    // Crear hilos del drone
    pthread_create(&drone->nav_thread, NULL, drone_navigation_thread, drone);
    pthread_create(&drone->fuel_thread, NULL, drone_fuel_thread, drone);
    pthread_create(&drone->comm_thread, NULL, drone_communication_thread, drone);
    pthread_create(&drone->payload_thread, NULL, drone_payload_thread, drone);
    
    log_message("Drone %d creado (Tipo: %s, Truck: %d, Swarm: %d)", 
               id, type == DRONE_TYPE_ATTACK ? "ATAQUE" : "CÁMARA", truck_id, swarm_id);
    
    return drone;
}

// Función para crear un enjambre
Swarm* create_swarm(int id, int truck_id, Position assembly_point, Position reassembly_point) {
    Swarm* swarm = malloc(sizeof(Swarm));
    if (!swarm) {
        log_message("Error: No se pudo asignar memoria para enjambre %d", id);
        return NULL;
    }
    
    swarm->id = id;
    swarm->truck_id = truck_id;
    swarm->ready_count = 0;
    swarm->active_count = 0;
    swarm->assembly_point = assembly_point;
    swarm->reassembly_point = reassembly_point;
    
    pthread_mutex_init(&swarm->mutex, NULL);
    
    // Crear drones del enjambre con distribución mixta de camiones
    // pero manteniendo la proporción 4:1 (ataque:cámara)
    for (int i = 0; i < DRONES_PER_SWARM; i++) {
        DroneType type = (i < ATTACK_DRONES_PER_SWARM) ? DRONE_TYPE_ATTACK : DRONE_TYPE_CAMERA;
        int drone_id = system_state.drone_count;
        
        // Distribuir drones aleatoriamente de diferentes camiones
        // pero manteniendo la proporción 4:1 (ataque:cámara)
        int source_truck;
        if (i == 4) {
            // El drone cámara siempre del camión principal para consistencia
            source_truck = truck_id;
        } else {
            // Los drones de ataque se distribuyen aleatoriamente entre los camiones
            source_truck = rand() % NUM_TRUCKS;
        }
        
        Position start_pos = system_state.trucks[source_truck].pos;
        
        swarm->drones[i] = create_drone(drone_id, source_truck, id, type, start_pos, assembly_point);
        if (swarm->drones[i]) {
            system_state.all_drones[system_state.drone_count++] = swarm->drones[i];
            swarm->active_count++;
        }
    }
    
    log_message("Enjambre %d creado con %d drones", id, swarm->active_count);
    return swarm;
}

// Función para inicializar el sistema
void initialize_system() {
    log_message("=== INICIANDO DRONE WARS 2 ===");
    
    // Inicializar mutex y condition variables
    pthread_mutex_init(&system_state.system_mutex, NULL);
    pthread_cond_init(&system_state.system_condition, NULL);
    pthread_mutex_init(&log_mutex, NULL);
    
    // Inicializar estado del sistema
    system_state.swarm_count = 0;
    system_state.drone_count = 0;
    system_state.event_count = 0;
    system_state.event_head = 0;
    system_state.event_tail = 0;
    system_state.global_attack_commanded = 0;
    system_state.all_swarms_ready = 0;
    system_state.simulation_running = 1;
    system_state.phase = 1;
    
    // Asignación aleatoria de objetivos a enjambres (para despistar al enemigo)
    system_state.target_assignments[0] = rand() % NUM_TARGETS;
    system_state.target_assignments[1] = rand() % NUM_TARGETS;
    system_state.target_assignments[2] = rand() % NUM_TARGETS;
    
    // Asegurar que no haya conflictos (cada enjambre ataque un objetivo diferente)
    while (system_state.target_assignments[1] == system_state.target_assignments[0]) {
        system_state.target_assignments[1] = rand() % NUM_TARGETS;
    }
    while (system_state.target_assignments[2] == system_state.target_assignments[0] || 
           system_state.target_assignments[2] == system_state.target_assignments[1]) {
        system_state.target_assignments[2] = rand() % NUM_TARGETS;
    }
    
    // Crear directorio FIFO
    mkdir(FIFO_PATH, 0777);
    
    // Crear FIFO del centro de comando
    snprintf(system_state.center_fifo_name, sizeof(system_state.center_fifo_name), 
             "%s/center", FIFO_PATH);
    mkfifo(system_state.center_fifo_name, 0666);
    system_state.center_fifo_fd = open(system_state.center_fifo_name, O_RDWR | O_NONBLOCK);
    
    // Inicializar posiciones fijas
    // Camiones
    system_state.trucks[0].pos = (Position){25, 0};
    system_state.trucks[1].pos = (Position){50, 0};
    system_state.trucks[2].pos = (Position){75, 0};
    
    // Objetivos
    system_state.targets[0].pos = (Position){0, 25};
    system_state.targets[1].pos = (Position){0, 50};
    system_state.targets[2].pos = (Position){0, 75};
    
    // Puntos de ensamblaje
    system_state.assembly_points[0] = (Position){25, 16};
    system_state.assembly_points[1] = (Position){50, 16};
    system_state.assembly_points[2] = (Position){75, 16};
    
    // Puntos de re-ensamblaje
    system_state.reassembly_points[0] = (Position){25, 82};
    system_state.reassembly_points[1] = (Position){50, 82};
    system_state.reassembly_points[2] = (Position){75, 82};
    
    // Defensas enemigas
    system_state.defenses[0].pos = (Position){10, 100};
    system_state.defenses[1].pos = (Position){90, 100};
    
    // Inicializar objetivos
    for (int i = 0; i < NUM_TARGETS; i++) {
        system_state.targets[i].id = i;
        system_state.targets[i].state = TARGET_STATE_INTACT;
        system_state.targets[i].attack_count = 0;
        system_state.targets[i].required_attacks = 4;
    }
    
    // Inicializar defensas
    for (int i = 0; i < NUM_ENEMY_DEFENSES; i++) {
        system_state.defenses[i].id = i;
        system_state.defenses[i].shoot_down_probability = system_state.W;
    }
    
    // Inicializar camiones
    for (int i = 0; i < NUM_TRUCKS; i++) {
        system_state.trucks[i].id = i;
        system_state.trucks[i].pos = system_state.trucks[i].pos;
        system_state.trucks[i].swarm = NULL;
        system_state.trucks[i].drone_count = 0;
        system_state.trucks[i].active = 1;
    }
    
    log_message("Sistema inicializado correctamente");
    
    // Mostrar asignación aleatoria de objetivos
    log_message("=== ASIGNACIÓN ALEATORIA DE OBJETIVOS ===");
    for (int i = 0; i < NUM_TRUCKS; i++) {
        log_message("Enjambre %d → Objetivo %d (Posición: %d,%d)", 
                   i, system_state.target_assignments[i],
                   system_state.targets[system_state.target_assignments[i]].pos.x,
                   system_state.targets[system_state.target_assignments[i]].pos.y);
    }
}

// Función para crear enjambres
void create_swarms() {
    log_message("=== FASE 1: CREANDO ENJAMBRES ===");
    
    for (int i = 0; i < NUM_TRUCKS; i++) {
        int swarm_id = system_state.swarm_count;
        Position assembly_point = system_state.assembly_points[i];
        Position reassembly_point = system_state.reassembly_points[i];
        
        Swarm* swarm = create_swarm(swarm_id, i, assembly_point, reassembly_point);
        if (swarm) {
            system_state.swarms[system_state.swarm_count++] = swarm;
            system_state.trucks[i].swarm = swarm;
            system_state.trucks[i].drone_count = DRONES_PER_SWARM;
        }
    }
    
    log_message("Se crearon %d enjambres", system_state.swarm_count);
}

// Función para optimizar la distribución de drones entre enjambres
void optimize_drone_distribution() {
    log_message("Optimizando distribución de drones entre enjambres...");
    
    // Permitir que cada enjambre pueda recibir drones de cualquier camión
    // manteniendo la proporción 4:1 (ataque:cámara)
    for (int i = 0; i < system_state.swarm_count; i++) {
        Swarm* swarm = system_state.swarms[i];
        if (swarm) {
            int needed_attack = 0;
            int needed_camera = 0;
            
            // Contar qué tipos de drones necesitamos
            for (int j = 0; j < DRONES_PER_SWARM; j++) {
                if (!swarm->drones[j] || swarm->drones[j]->state == DRONE_STATE_DESTROYED) {
                    if (j < ATTACK_DRONES_PER_SWARM) {
                        needed_attack++;
                    } else {
                        needed_camera++;
                    }
                }
            }
            
            if (needed_attack > 0 || needed_camera > 0) {
                log_message("Enjambre %d necesita %d drones de ataque y %d drones cámara", 
                           i, needed_attack, needed_camera);
                
                // Buscar drones disponibles en otros enjambres
                for (int k = 0; k < system_state.swarm_count; k++) {
                    if (k != i && system_state.swarms[k] && system_state.swarms[k]->active_count > DRONES_PER_SWARM) {
                        Swarm* other_swarm = system_state.swarms[k];
                        pthread_mutex_lock(&other_swarm->mutex);
                        
                        for (int l = 0; l < DRONES_PER_SWARM; l++) {
                            if (other_swarm->drones[l] && 
                                other_swarm->drones[l]->state == DRONE_STATE_READY) {
                                
                                // Verificar si necesitamos este tipo de drone
                                DroneType drone_type = other_swarm->drones[l]->type;
                                if ((drone_type == DRONE_TYPE_ATTACK && needed_attack > 0) ||
                                    (drone_type == DRONE_TYPE_CAMERA && needed_camera > 0)) {
                                    
                                    // Transferir drone
                                    other_swarm->drones[l]->swarm_id = i;
                                    other_swarm->drones[l]->target = swarm->assembly_point;
                                    other_swarm->drones[l]->state = DRONE_STATE_FLYING_TO_ASSEMBLY;
                                    
                                    // Encontrar posición vacía en el enjambre destino
                                    for (int m = 0; m < DRONES_PER_SWARM; m++) {
                                        if (!swarm->drones[m] || swarm->drones[m]->state == DRONE_STATE_DESTROYED) {
                                            swarm->drones[m] = other_swarm->drones[l];
                                            other_swarm->drones[l] = NULL;
                                            other_swarm->active_count--;
                                            swarm->active_count++;
                                            
                                            if (drone_type == DRONE_TYPE_ATTACK) needed_attack--;
                                            else needed_camera--;
                                            
                                            log_message("Drone %d (Tipo: %s, Truck: %d) transferido del enjambre %d al enjambre %d", 
                                                       swarm->drones[m]->id, 
                                                       drone_type == DRONE_TYPE_ATTACK ? "ATAQUE" : "CÁMARA",
                                                       swarm->drones[m]->truck_id, k, i);
                                            break;
                                        }
                                    }
                                    
                                    if (needed_attack == 0 && needed_camera == 0) break;
                                }
                            }
                        }
                        
                        pthread_mutex_unlock(&other_swarm->mutex);
                        if (needed_attack == 0 && needed_camera == 0) break;
                    }
                }
            }
        }
    }
    
    log_message("Optimización de distribución completada");
}

// Función para esperar a que todos los enjambres estén listos
void wait_for_all_swarms_ready() {
    log_message("=== ESPERANDO A QUE TODOS LOS ENJAMBRES ESTÉN LISTOS ===");
    
    while (system_state.simulation_running) {
        int all_ready = 1;
        
        for (int i = 0; i < system_state.swarm_count; i++) {
            Swarm* swarm = system_state.swarms[i];
            if (swarm && swarm->active_count > 0) {
                int swarm_ready = 0;
                pthread_mutex_lock(&swarm->mutex);
                
                for (int j = 0; j < DRONES_PER_SWARM; j++) {
                    if (swarm->drones[j] && 
                        (swarm->drones[j]->state == DRONE_STATE_READY || 
                         swarm->drones[j]->state == DRONE_STATE_CIRCLING_ASSEMBLY)) {
                        swarm_ready++;
                    }
                }
                
                swarm->ready_count = swarm_ready;
                if (swarm_ready < swarm->active_count) {
                    all_ready = 0;
                }
                
                pthread_mutex_unlock(&swarm->mutex);
            }
        }
        
        if (all_ready) {
            system_state.all_swarms_ready = 1;
            log_message("Todos los enjambres están listos para el ataque");
            break;
        }
        
        usleep(500000); // 500ms
    }
}

// Función para comandar ataque global
void command_global_attack() {
    log_message("=== FASE 2: COMANDANDO ATAQUE GLOBAL ===");
    
    system_state.phase = 2;
    system_state.global_attack_commanded = 1;
    
    // Enviar comando a todos los enjambres
    for (int i = 0; i < system_state.swarm_count; i++) {
        Swarm* swarm = system_state.swarms[i];
        if (swarm && swarm->active_count > 0) {
            // Los drones van al objetivo asignado aleatoriamente
            int target_id = system_state.target_assignments[i];
            for (int j = 0; j < DRONES_PER_SWARM; j++) {
                if (swarm->drones[j] && 
                    (swarm->drones[j]->state == DRONE_STATE_READY || 
                     swarm->drones[j]->state == DRONE_STATE_CIRCLING_ASSEMBLY)) {
                    
                    // Si está volando en círculos, cambiar a READY primero
                    if (swarm->drones[j]->state == DRONE_STATE_CIRCLING_ASSEMBLY) {
                        swarm->drones[j]->state = DRONE_STATE_READY;
                        log_message("Drone %d terminó patrulla circular, listo para ataque", swarm->drones[j]->id);
                    }
                    
                    swarm->drones[j]->target = system_state.targets[target_id].pos;
                    swarm->drones[j]->state = DRONE_STATE_FLYING_TO_TARGET;
                }
            }
            
            log_message("Enjambre %d asignado al objetivo %d", i, target_id);
        }
    }
    
    log_message("Comando de ataque global enviado a todos los enjambres");
}

// Función para esperar a que todos los drones crucen la zona de defensa
void wait_for_defense_zone_crossing() {
    log_message("=== FASE 3: CRUZANDO ZONA DE DEFENSA ===");
    
    system_state.phase = 3;
    
    int drones_in_defense_zone = 1;
    int max_wait_time = 20;
    int wait_time = 0;
    
    log_message("Monitoreando cruce de zona de defensa...");
    
    while (drones_in_defense_zone > 0 && wait_time < max_wait_time) {
        drones_in_defense_zone = 0;
        
        // Contar drones que aún están en la zona de defensa
        for (int i = 0; i < system_state.swarm_count; i++) {
            Swarm* swarm = system_state.swarms[i];
            if (swarm && swarm->active_count > 0) {
                pthread_mutex_lock(&swarm->mutex);
                
                for (int j = 0; j < DRONES_PER_SWARM; j++) {
                    if (swarm->drones[j] && 
                        swarm->drones[j]->state == DRONE_STATE_FLYING_TO_TARGET &&
                        is_drone_in_zone(swarm->drones[j], DEFENSE_ZONE_START, DEFENSE_ZONE_END)) {
                        drones_in_defense_zone++;
                    }
                }
                
                pthread_mutex_unlock(&swarm->mutex);
            }
        }
        
        if (drones_in_defense_zone > 0) {
            // Solo mostrar mensaje cada 5 segundos para evitar spam
            if (wait_time % 5 == 0) {
                log_message("Esperando... %d drones aún en zona de defensa", drones_in_defense_zone);
            }
            sleep(1);
            wait_time++;
        }
    }
    
    if (wait_time >= max_wait_time) {
        log_message("Tiempo de espera agotado, continuando...");
    } else {
        log_message("Todos los drones están cruzando la zona de defensa");
    }
}

// Función para esperar a que todos los drones lleguen al punto de re-ensamblaje
void wait_for_reassembly_ready() {
    log_message("=== FASE 3.1: ESPERANDO RE-ENSAMBLAJE ===");
    
    system_state.phase = 31;
    
    int drones_not_at_reassembly = 1;
    int max_wait_time = 20;
    int wait_time = 0;
    
    log_message("Esperando a que todos los drones lleguen al punto de re-ensamblaje...");
    
    while (drones_not_at_reassembly > 0 && wait_time < max_wait_time) {
        drones_not_at_reassembly = 0;
        
        // Contar drones que aún no han llegado al punto de re-ensamblaje
        for (int i = 0; i < system_state.swarm_count; i++) {
            Swarm* swarm = system_state.swarms[i];
            if (swarm && swarm->active_count > 0) {
                pthread_mutex_lock(&swarm->mutex);
                
                int ready_count = 0;
                int flying_count = 0;
                int destroyed_count = 0;
                
                for (int j = 0; j < DRONES_PER_SWARM; j++) {
                    if (swarm->drones[j]) {
                        if (swarm->drones[j]->state == DRONE_STATE_READY) {
                            ready_count++;
                        } else if (swarm->drones[j]->state == DRONE_STATE_FLYING_TO_TARGET) {
                            flying_count++;
                        } else if (swarm->drones[j]->state == DRONE_STATE_DESTROYED) {
                            destroyed_count++;
                        } else {
                            drones_not_at_reassembly++;
                        }
                    }
                }
                
                pthread_mutex_unlock(&swarm->mutex);
                
                // Mostrar estado detallado cada 5 segundos
                if (wait_time % 5 == 0) {
                    log_message("Enjambre %d: %d READY, %d volando, %d destruidos, %d pendientes", 
                               i, ready_count, flying_count, destroyed_count, drones_not_at_reassembly);
                }
            }
        }
        
        if (drones_not_at_reassembly > 0) {
            // Solo mostrar mensaje cada 5 segundos para evitar spam
            if (wait_time % 5 == 0) {
                log_message("Esperando... %d drones aún no han llegado al re-ensamblaje", drones_not_at_reassembly);
            }
            sleep(1);
            wait_time++;
        }
    }
    
    if (wait_time >= max_wait_time) {
        log_message("Tiempo de espera agotado, continuando...");
    } else {
        log_message("¡Todos los drones han llegado al punto de re-ensamblaje!");
    }
    
    system_state.phase = 4;
}

// Función para enviar drones al ataque final
void command_final_attack() {
    log_message("=== FASE 4: ENVIANDO DRONES AL ATAQUE FINAL ===");
    
    system_state.phase = 4;
    
    // Los drones ya están en camino al objetivo desde la Fase 2
    // Esta función solo confirma que continúan
    log_message("Los drones ya están en camino al objetivo desde la Fase 2");
    log_message("Comando de ataque final confirmado");
}

// Función para esperar a que todos los drones lleguen al objetivo
void wait_for_all_drones_at_target() {
    log_message("=== FASE 4.1: ESPERANDO A QUE TODOS LLEGUEN AL OBJETIVO ===");
    
    system_state.phase = 41;
    
    int drones_not_at_target = 1;
    int max_wait_time = 30; // Aumentar tiempo de espera
    int wait_time = 0;
    
    log_message("Esperando a que todos los drones lleguen al objetivo...");
    
    while (drones_not_at_target > 0 && wait_time < max_wait_time) {
        drones_not_at_target = 0;
        
        // Contar drones que aún no han llegado al objetivo
        for (int i = 0; i < system_state.swarm_count; i++) {
            Swarm* swarm = system_state.swarms[i];
            if (swarm && swarm->active_count > 0) {
                pthread_mutex_lock(&swarm->mutex);
                
                int at_target_count = 0;
                int flying_count = 0;
                int destroyed_count = 0;
                
                for (int j = 0; j < DRONES_PER_SWARM; j++) {
                    if (swarm->drones[j]) {
                        if (swarm->drones[j]->state == DRONE_STATE_AT_TARGET) {
                            at_target_count++;
                        } else if (swarm->drones[j]->state == DRONE_STATE_FLYING_TO_TARGET) {
                            flying_count++;
                            drones_not_at_target++;
                        } else if (swarm->drones[j]->state == DRONE_STATE_DESTROYED) {
                            destroyed_count++;
                        } else {
                            drones_not_at_target++;
                        }
                    }
                }
                
                pthread_mutex_unlock(&swarm->mutex);
                
                // Mostrar estado detallado cada 5 segundos
                if (wait_time % 5 == 0) {
                    log_message("Enjambre %d: %d en objetivo, %d volando, %d destruidos", 
                               i, at_target_count, flying_count, destroyed_count);
                }
            }
        }
        
        if (drones_not_at_target > 0) {
            // Solo mostrar mensaje cada 5 segundos para evitar spam
            if (wait_time % 5 == 0) {
                log_message("Esperando... %d drones aún no han llegado al objetivo", drones_not_at_target);
            }
            sleep(1);
            wait_time++;
        }
    }
    
    if (wait_time >= max_wait_time) {
        log_message("Tiempo de espera agotado, continuando con detonación...");
    } else {
        log_message("¡Todos los drones supervivientes han llegado al objetivo!");
    }
    
    system_state.phase = 5;
}

// Función para enviar comando de detonación a todos los drones de ataque
void command_detonation() {
    log_message("=== FASE 5: ENVIANDO COMANDO DE DETONACIÓN ===");
    
    system_state.phase = 5;
    
    // Enviar comando de detonación a todos los drones de ataque que están en el objetivo
    for (int i = 0; i < system_state.swarm_count; i++) {
        Swarm* swarm = system_state.swarms[i];
        if (swarm && swarm->active_count > 0) {
            pthread_mutex_lock(&swarm->mutex);
            
            for (int j = 0; j < DRONES_PER_SWARM; j++) {
                if (swarm->drones[j] && 
                    swarm->drones[j]->state == DRONE_STATE_AT_TARGET) {
                    
                    if (swarm->drones[j]->type == DRONE_TYPE_ATTACK) {
                        // Enviar comando de detonación a drones de ataque
                        swarm->drones[j]->state = DRONE_STATE_DETONATED;
                        log_message("Drone de ataque %d detonó en objetivo", swarm->drones[j]->id);
                        send_event(EVT_DETONATED, swarm->drones[j]->id, swarm->drones[j]->swarm_id, swarm->drones[j]->truck_id, "DETONATED");
                        
                        // Marcar drone como detonado (no destruido inmediatamente)
                        swarm->drones[j]->state = DRONE_STATE_DETONATED;
                        
                    } else if (swarm->drones[j]->type == DRONE_TYPE_CAMERA) {
                        // Drone cámara hace reporte final del estado del objetivo
                        log_message("Drone cámara %d haciendo reporte final del estado del objetivo", swarm->drones[j]->id);
                        
                        // Contar drones de ataque que detonaron para determinar estado del objetivo
                        int detonated_attack = 0;
                        for (int k = 0; k < ATTACK_DRONES_PER_SWARM; k++) {
                            if (swarm->drones[k] && swarm->drones[k]->state == DRONE_STATE_DETONATED) {
                                detonated_attack++;
                            }
                        }
                        
                        // Determinar estado del objetivo según drones que detonaron
                        const char* target_status;
                        if (detonated_attack >= 4) {
                            target_status = "OBJETIVO DESTRUIDO";
                        } else if (detonated_attack > 0) {
                            target_status = "OBJETIVO PARCIALMENTE DESTRUIDO";
                        } else {
                            target_status = "OBJETIVO INTACTO";
                        }
                        
                        // Enviar reporte con el estado del objetivo
                        send_event(EVT_CAM_REPORT_OK, swarm->drones[j]->id, swarm->drones[j]->swarm_id, swarm->drones[j]->truck_id, target_status);
                        
                        log_message("Drone cámara %d reporta: %s (%d drones de ataque detonaron)", 
                                   swarm->drones[j]->id, target_status, detonated_attack);
                        
                        // Drone cámara completa misión y se autodestruye después del reporte
                        swarm->drones[j]->state = DRONE_STATE_MISSION_COMPLETE;
                        log_message("Drone cámara %d se autodestruye después de completar su misión", swarm->drones[j]->id);
                    }
                }
            }
            
            pthread_mutex_unlock(&swarm->mutex);
        }
    }
    
    log_message("Comando de detonación enviado a todos los drones de ataque");
    
    // ESTADO FINAL: Mostrar estado de cada enjambre, cada dron y estado del objetivo
    log_message("=== ESTADO FINAL DE LA SIMULACIÓN ===");
    
    // Mostrar estado de cada objetivo
    log_message("=== ESTADO DE LOS OBJETIVOS ===");
    for (int i = 0; i < NUM_TARGETS; i++) {
        // Buscar qué enjambre ataca este objetivo según las asignaciones aleatorias
        int attacking_swarm = -1;
        for (int j = 0; j < system_state.swarm_count; j++) {
            if (system_state.target_assignments[j] == i) {
                attacking_swarm = j;
                break;
            }
        }
        
        if (attacking_swarm >= 0) {
            Swarm* swarm = system_state.swarms[attacking_swarm];
        if (swarm && swarm->active_count > 0) {
            int detonated_attack = 0;
            int camera_active = 0;
            
            pthread_mutex_lock(&swarm->mutex);
            
            // Contar drones de ataque que detonaron en ESTE enjambre
            for (int k = 0; k < ATTACK_DRONES_PER_SWARM; k++) {
                if (swarm->drones[k] && swarm->drones[k]->state == DRONE_STATE_DETONATED) {
                    detonated_attack++;
                }
            }
            
            // Verificar si el drone cámara completó su misión (no fue destruido por torretas)
            for (int k = ATTACK_DRONES_PER_SWARM; k < DRONES_PER_SWARM; k++) {
                if (swarm->drones[k] && swarm->drones[k]->state == DRONE_STATE_MISSION_COMPLETE) {
                    camera_active = 1;
                }
            }
            
            pthread_mutex_unlock(&swarm->mutex);
            
            // Determinar estado del objetivo
            const char* target_status;
            const char* confirmation_status = camera_active ? "CONFIRMADO" : "SIN CONFIRMAR";
            
            if (detonated_attack >= 4) {
                target_status = "DESTRUIDO";
            } else if (detonated_attack > 0) {
                target_status = "PARCIALMENTE DESTRUIDO";
            } else {
                target_status = "INTACTO";
            }
            
            log_message("Objetivo %d: %s %s (%d drones de ataque detonaron) - Atacado por enjambre %d", 
                       i, target_status, confirmation_status, detonated_attack, attacking_swarm);
        } else {
            log_message("Objetivo %d: ESTADO DESCONOCIDO (enjambre %d destruido)", i, attacking_swarm);
        }
        } else {
            log_message("Objetivo %d: SIN ASIGNAR", i);
        }
    }
    
    log_message("=== ESTADO DE LOS ENJAMBRES ===");
    
    for (int i = 0; i < system_state.swarm_count; i++) {
        Swarm* swarm = system_state.swarms[i];
        if (swarm && swarm->active_count > 0) {
            int attack_drones = 0;
            int camera_drones = 0;
            int destroyed_attack = 0;
            int destroyed_camera = 0;
            
            pthread_mutex_lock(&swarm->mutex);
            
            // Contar estado de cada dron en el enjambre
            for (int j = 0; j < DRONES_PER_SWARM; j++) {
                if (swarm->drones[j]) {
                    if (j < ATTACK_DRONES_PER_SWARM) {
                        if (swarm->drones[j]->state == DRONE_STATE_DESTROYED) {
                            destroyed_attack++;
                        } else if (swarm->drones[j]->state == DRONE_STATE_DETONATED) {
                            destroyed_attack++; // Contar detonados como destruidos para el conteo
                        } else {
                            attack_drones++;
                        }
                    } else {
                        if (swarm->drones[j]->state == DRONE_STATE_DESTROYED) {
                            destroyed_camera++;
                        } else if (swarm->drones[j]->state == DRONE_STATE_MISSION_COMPLETE) {
                            destroyed_camera++; // Contar como "destruido" para el conteo pero era misión completada
                        } else {
                            camera_drones++;
                        }
                    }
                }
            }
            
            pthread_mutex_unlock(&swarm->mutex);
            
            // Mostrar estado del enjambre
            log_message("Enjambre %d: %d ataque activos, %d cámara activos, %d ataque destruidos, %d cámara destruidos", 
                       i, attack_drones, camera_drones, destroyed_attack, destroyed_camera);
            
            // Mostrar estado de cada dron individual
            pthread_mutex_lock(&swarm->mutex);
            for (int j = 0; j < DRONES_PER_SWARM; j++) {
                if (swarm->drones[j]) {
                    const char* drone_type = (j < ATTACK_DRONES_PER_SWARM) ? "ATAQUE" : "CÁMARA";
                    const char* drone_status;
                    
                    // Estados más claros y descriptivos
                    switch (swarm->drones[j]->state) {
                        case DRONE_STATE_DESTROYED:
                            drone_status = "DESTRUIDO";
                            break;
                        case DRONE_STATE_DETONATED:
                            drone_status = "DETONÓ EN OBJETIVO";
                            break;
                        case DRONE_STATE_MISSION_COMPLETE:
                            drone_status = "MISIÓN COMPLETADA";
                            break;
                        case DRONE_STATE_AT_TARGET:
                            drone_status = "EN OBJETIVO";
                            break;
                        case DRONE_STATE_FLYING_TO_TARGET:
                            drone_status = "VOLANDO AL OBJETIVO";
                            break;
                        case DRONE_STATE_READY:
                            drone_status = "EN PUNTO DE ENSAMBLAJE";
                            break;
                        case DRONE_STATE_CIRCLING_ASSEMBLY:
                            drone_status = "PATRULLANDO EN CÍRCULOS";
                            break;
                        default:
                            drone_status = "ESTADO DESCONOCIDO";
                            break;
                    }
                    
                    log_message("  Drone %d (%s): %s", swarm->drones[j]->id, drone_type, drone_status);
                }
            }
            pthread_mutex_unlock(&swarm->mutex);
        }
    }
    
    log_message("Estado final de la simulación completado");
}

// Función auxiliar para extraer drones de un enjambre específico
int try_extract_drones_from_swarm(Swarm* target_swarm, int source_swarm_id, int* needed_attack, int* needed_camera, int target_swarm_id) {
    Swarm* source_swarm = system_state.swarms[source_swarm_id];
    if (!source_swarm || source_swarm->active_count <= DRONES_PER_SWARM) {
        return 0; // No se puede extraer de enjambres completos
    }
    
    int drones_extracted = 0;
    pthread_mutex_lock(&source_swarm->mutex);
    
    // Buscar drones disponibles del tipo que necesitamos
    for (int j = 0; j < DRONES_PER_SWARM && (*needed_attack > 0 || *needed_camera > 0); j++) {
        if (source_swarm->drones[j] && source_swarm->drones[j]->state == DRONE_STATE_READY) {
            DroneType drone_type = source_swarm->drones[j]->type;
            
            if ((drone_type == DRONE_TYPE_ATTACK && *needed_attack > 0) ||
                (drone_type == DRONE_TYPE_CAMERA && *needed_camera > 0)) {
                
                // Transferir drone
                source_swarm->drones[j]->swarm_id = target_swarm_id;
                source_swarm->drones[j]->target = target_swarm->reassembly_point;
                source_swarm->drones[j]->state = DRONE_STATE_FLYING_TO_ASSEMBLY;
                
                // Encontrar posición vacía en el enjambre destino
                for (int m = 0; m < DRONES_PER_SWARM; m++) {
                    if (!target_swarm->drones[m] || target_swarm->drones[m]->state == DRONE_STATE_DESTROYED) {
                        target_swarm->drones[m] = source_swarm->drones[j];
                        source_swarm->drones[j] = NULL;
                        source_swarm->active_count--;
                        target_swarm->active_count++;
                        
                        if (drone_type == DRONE_TYPE_ATTACK) (*needed_attack)--;
                        else (*needed_camera)--;
                        
                        drones_extracted++;
                        
                        log_message("Drone %d (Tipo: %s) transferido del enjambre %d al enjambre %d", 
                                   target_swarm->drones[m]->id, 
                                   drone_type == DRONE_TYPE_ATTACK ? "ATAQUE" : "CÁMARA", 
                                   source_swarm_id, target_swarm_id);
                        break;
                    }
                }
            }
        }
    }
    
    pthread_mutex_unlock(&source_swarm->mutex);
    return drones_extracted;
}

// Función para manejar re-ensamblaje
void handle_reassembly() {
    log_message("=== FASE 4: MANEJANDO RE-ENSAMBLAJE ===");
    
    system_state.phase = 4;
    
    // Primera pasada: identificar enjambres incompletos y completos
    int incomplete_swarms[MAX_DRONES / DRONES_PER_SWARM];
    int complete_swarms[MAX_DRONES / DRONES_PER_SWARM];
    int incomplete_count = 0;
    int complete_count = 0;
    
    for (int i = 0; i < system_state.swarm_count; i++) {
        Swarm* swarm = system_state.swarms[i];
        if (swarm && swarm->active_count > 0) {
            int active_drones = 0;
            int attack_drones = 0;
            int camera_drones = 0;
            
            pthread_mutex_lock(&swarm->mutex);
            
            for (int j = 0; j < DRONES_PER_SWARM; j++) {
                if (swarm->drones[j] && swarm->drones[j]->state != DRONE_STATE_DESTROYED) {
                    active_drones++;
                    if (j < ATTACK_DRONES_PER_SWARM) {
                        attack_drones++;
                    } else {
                        camera_drones++;
                    }
                }
            }
            
            // Un enjambre está completo si tiene exactamente 4 drones de ataque y 1 cámara
            if (attack_drones == ATTACK_DRONES_PER_SWARM && camera_drones == 1) {
                complete_swarms[complete_count++] = i;
                log_message("Enjambre %d COMPLETO (%d ataque, %d cámara)", i, attack_drones, camera_drones);
            } else {
                incomplete_swarms[incomplete_count++] = i;
                log_message("Enjambre %d INCOMPLETO (%d ataque, %d cámara) - necesita %d ataque, %d cámara", 
                           i, attack_drones, camera_drones, 
                           ATTACK_DRONES_PER_SWARM - attack_drones, 1 - camera_drones);
            }
            
            pthread_mutex_unlock(&swarm->mutex);
        }
    }
    
    log_message("Enjambres incompletos: %d, Enjambres completos: %d", incomplete_count, complete_count);
    
    // Segunda pasada: re-ensamblar cada enjambre incompleto de forma independiente
    for (int idx = 0; idx < incomplete_count; idx++) {
        int i = incomplete_swarms[idx];
        Swarm* swarm = system_state.swarms[i];
        
        if (!swarm) continue;
        
        pthread_mutex_lock(&swarm->mutex);
        
        // Contar qué tipos de drones necesitamos
        int needed_attack = 0;
        int needed_camera = 0;
        
        for (int j = 0; j < DRONES_PER_SWARM; j++) {
            if (!swarm->drones[j] || swarm->drones[j]->state == DRONE_STATE_DESTROYED) {
                if (j < ATTACK_DRONES_PER_SWARM) {
                    needed_attack++;
                } else {
                    needed_camera++;
                }
            }
        }
        
        if (needed_attack > 0 || needed_camera > 0) {
            log_message("Enjambre %d necesita %d drones de ataque y %d drones cámara", 
                       i, needed_attack, needed_camera);
            
            // Búsqueda alternada: primero enjambres más cercanos, luego más lejanos
            int search_radius = 1;
            int drones_found = 0;
            
            while (drones_found < (needed_attack + needed_camera) && search_radius < system_state.swarm_count) {
                // Buscar en enjambres a izquierda y derecha
                int left_swarm = i - search_radius;
                int right_swarm = i + search_radius;
                
                // Verificar límites del array
                if (left_swarm >= 0) {
                    drones_found += try_extract_drones_from_swarm(swarm, left_swarm, &needed_attack, &needed_camera, i);
                }
                
                if (right_swarm < system_state.swarm_count) {
                    drones_found += try_extract_drones_from_swarm(swarm, right_swarm, &needed_attack, &needed_camera, i);
                }
                
                search_radius++;
            }
            
            log_message("Enjambre %d completado con %d drones transferidos", i, drones_found);
        }
        
        pthread_mutex_unlock(&swarm->mutex);
    }
    
    // Tercera pasada: cambiar estado de todos los drones a REASSEMBLED
    log_message("Cambiando estado de todos los drones a REASSEMBLED...");
    for (int i = 0; i < system_state.swarm_count; i++) {
        Swarm* swarm = system_state.swarms[i];
        if (swarm && swarm->active_count > 0) {
            pthread_mutex_lock(&swarm->mutex);
            
            for (int j = 0; j < DRONES_PER_SWARM; j++) {
                if (swarm->drones[j] && swarm->drones[j]->state == DRONE_STATE_READY) {
                    swarm->drones[j]->state = DRONE_STATE_REASSEMBLED;
                }
            }
            
            pthread_mutex_unlock(&swarm->mutex);
        }
    }
    
    log_message("Re-ensamblaje completado, todos los drones están listos para el ataque final");
    
    // Mostrar estado final de cada enjambre
    for (int i = 0; i < system_state.swarm_count; i++) {
        Swarm* swarm = system_state.swarms[i];
        if (swarm && swarm->active_count > 0) {
            int attack_count = 0;
            int camera_count = 0;
            int destroyed_count = 0;
            
            pthread_mutex_lock(&swarm->mutex);
            
            for (int j = 0; j < DRONES_PER_SWARM; j++) {
                if (swarm->drones[j]) {
                    if (swarm->drones[j]->state == DRONE_STATE_DESTROYED) {
                        destroyed_count++;
                    } else if (j < ATTACK_DRONES_PER_SWARM) {
                        attack_count++;
                    } else {
                        camera_count++;
                    }
                }
            }
            
            pthread_mutex_unlock(&swarm->mutex);
            
            log_message("Enjambre %d final: %d ataque, %d cámara, %d destruidos", 
                       i, attack_count, camera_count, destroyed_count);
        }
    }
}

// Función para procesar eventos
void process_events() {
    while (system_state.event_count > 0) {
        pthread_mutex_lock(&system_state.system_mutex);
        
        Event* event = &system_state.event_queue[system_state.event_head];
        system_state.event_head = (system_state.event_head + 1) % MAX_EVENTS;
        system_state.event_count--;
        
        pthread_mutex_unlock(&system_state.system_mutex);
        
        // Procesar evento según su tipo
        switch (event->type) {
            case EVT_READY:
                log_message("Drone %d reporta READY", event->drone_id);
                break;
                
            case EVT_AT_TARGET:
                log_message("Drone %d llegó al objetivo", event->drone_id);
                break;
                
            case EVT_DETONATED:
                log_message("Drone %d detonó exitosamente", event->drone_id);
                // Incrementar contador de ataques al objetivo
                break;
                
            case EVT_CAM_REPORT_OK:
                log_message("Drone cámara %d reporta: %s", event->drone_id, event->data);
                break;
                
            case EVT_CAM_REPORT_FAIL:
                log_message("Drone cámara %d falló en reportar", event->drone_id);
                break;
                
            case EVT_DESTROYED:
                log_message("Drone %d fue destruido", event->drone_id);
                break;
                
            case EVT_FUEL_EMPTY:
                log_message("Drone %d se quedó sin combustible", event->drone_id);
                break;
        }
    }
}

// Función para verificar estado de la simulación
void check_simulation_status() {
    int active_drones = 0;
    int completed_swarms = 0;
    
    for (int i = 0; i < system_state.swarm_count; i++) {
        Swarm* swarm = system_state.swarms[i];
        if (swarm) {
            int swarm_active = 0;
            for (int j = 0; j < DRONES_PER_SWARM; j++) {
                if (swarm->drones[j] && swarm->drones[j]->state != DRONE_STATE_DESTROYED) {
                    swarm_active++;
                }
            }
            if (swarm_active == 0) {
                completed_swarms++;
            }
            active_drones += swarm_active;
        }
    }
    
    // Solo marcar como completada si ya pasamos por la fase de detonación
    if (completed_swarms == system_state.swarm_count && system_state.phase >= 5) {
        log_message("=== SIMULACIÓN COMPLETADA ===");
        system_state.simulation_running = 0;
    }
    
    // Solo mostrar estado cuando hay cambios significativos
    static int last_active_drones = -1;
    static int last_completed_swarms = -1;
    
    if (active_drones != last_active_drones || completed_swarms != last_completed_swarms) {
        log_message("Estado: %d drones activos, %d enjambres completados", 
                   active_drones, completed_swarms);
        last_active_drones = active_drones;
        last_completed_swarms = completed_swarms;
    }
}

// Función principal del centro de comando
void command_center() {
    log_message("Centro de Comando iniciado");
    
    // ===== FASE 1: ENSAMBLAJE Y OPTIMIZACIÓN =====
    log_message("=== INICIANDO FASE 1: ENSAMBLAJE Y OPTIMIZACIÓN ===");
    create_swarms();
    
    // Esperar a que todos los enjambres estén listos
    wait_for_all_swarms_ready();
    
    // ===== FASE 2: ATAQUE Y CRUCE DE ZONA DE DEFENSA =====
    log_message("=== INICIANDO FASE 2: ATAQUE Y CRUCE DE ZONA DE DEFENSA ===");
    command_global_attack();
    
    // ===== FASE 3: CRUZANDO ZONA DE DEFENSA =====
    wait_for_defense_zone_crossing();
    
    // ===== FASE 4: ATAQUE FINAL =====
    log_message("=== INICIANDO FASE 4: ATAQUE FINAL ===");
    command_final_attack();
    
    // ===== FASE 4.1: ESPERANDO A QUE TODOS LLEGUEN AL OBJETIVO =====
    wait_for_all_drones_at_target();
    
    // ===== FASE 5: DETONACIÓN =====
    log_message("=== INICIANDO FASE 5: DETONACIÓN ===");
    command_detonation();
    
    // Esperar a que se complete la detonación
    log_message("Esperando a que se complete la detonación...");
    sleep(2);
    
    // Marcar simulación como completada después de la detonación
    log_message("Simulación completada. Terminando...");
    system_state.simulation_running = 0;
    
    // Procesar eventos finales una vez más
    process_events();
    
    log_message("Centro de Comando finalizado");
}

// Función para cargar configuración
void load_configuration() {
    FILE* config_file = fopen("config.txt", "r");
    if (!config_file) {
        log_message("Error: No se pudo abrir config.txt, usando valores por defecto");
        system_state.W = 30;
        system_state.Q = 10;
        system_state.Z = 4;
        system_state.speed = 2;
        system_state.initial_fuel = 100;
        system_state.ticks = 1000;
        return;
    }
    
    char line[256];
    while (fgets(line, sizeof(line), config_file)) {
        if (strncmp(line, "W=", 2) == 0) {
            system_state.W = atoi(line + 2);
        } else if (strncmp(line, "Q=", 2) == 0) {
            system_state.Q = atoi(line + 2);
        } else if (strncmp(line, "Z=", 2) == 0) {
            system_state.Z = atoi(line + 2);
        } else if (strncmp(line, "speed=", 6) == 0) {
            system_state.speed = atoi(line + 6);
        } else if (strncmp(line, "fuel=", 5) == 0) {
            system_state.initial_fuel = atoi(line + 5);
        } else if (strncmp(line, "ticks=", 6) == 0) {
            system_state.ticks = atoi(line + 6);
        }
    }
    
    fclose(config_file);
    
    log_message("Configuración cargada: W=%d%%, Q=%d%%, Z=%ds, speed=%d, fuel=%d, ticks=%d",
               system_state.W, system_state.Q, system_state.Z, system_state.speed, 
               system_state.initial_fuel, system_state.ticks);
}

// Función para limpiar recursos
void cleanup_system() {
    log_message("Limpiando recursos del sistema...");
    
    // Detener todos los drones
    for (int i = 0; i < system_state.drone_count; i++) {
        if (system_state.all_drones[i]) {
            system_state.all_drones[i]->active = 0;
            
            // Esperar a que terminen los hilos
            pthread_join(system_state.all_drones[i]->nav_thread, NULL);
            pthread_join(system_state.all_drones[i]->fuel_thread, NULL);
            pthread_join(system_state.all_drones[i]->comm_thread, NULL);
            pthread_join(system_state.all_drones[i]->payload_thread, NULL);
            
            // Cerrar FIFO
            if (system_state.all_drones[i]->fifo_fd != -1) {
                close(system_state.all_drones[i]->fifo_fd);
            }
            
            // Destruir mutex y condition variable
            pthread_mutex_destroy(&system_state.all_drones[i]->mutex);
            pthread_cond_destroy(&system_state.all_drones[i]->condition);
            
            // Liberar memoria
            free(system_state.all_drones[i]);
        }
    }
    
    // Liberar enjambres
    for (int i = 0; i < system_state.swarm_count; i++) {
        if (system_state.swarms[i]) {
            pthread_mutex_destroy(&system_state.swarms[i]->mutex);
            free(system_state.swarms[i]);
        }
    }
    
    // Cerrar FIFO del centro
    if (system_state.center_fifo_fd != -1) {
        close(system_state.center_fifo_fd);
    }
    
    // Destruir mutex y condition variables del sistema
    pthread_mutex_destroy(&system_state.system_mutex);
    pthread_cond_destroy(&system_state.system_condition);
    pthread_mutex_destroy(&log_mutex);
    
    log_message("Recursos del sistema limpiados");
}

// Función principal
int main() {
    // Inicializar generador de números aleatorios
    srand(time(NULL));
    
    // Cargar configuración
    load_configuration();
    
    // Inicializar sistema
    initialize_system();
    
    // Ejecutar centro de comando
    command_center();
    
    // Limpiar recursos
    cleanup_system();
    
    log_message("=== DRONE WARS 2 FINALIZADO ===");
    
    return 0;
}
