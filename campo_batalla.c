
// Incluir stdio.h primero para evitar warnings de printf
#include <stdio.h>
#include <sys/stat.h>
#include <string.h>
// FIFO global para comunicación de drones al centro de control
#define FIFO_CENTRO "/tmp/fifo_centro"
#include <semaphore.h>
#include <fcntl.h>
// Nombres de semáforos para sincronización de enjambres
#define SEM_ENJAMBRE1 "/sem_enjambre1"
#define SEM_ENJAMBRE2 "/sem_enjambre2"

// 4 drones de ataque y 1 de cámara por enjambre
#define DRONES_ATAQUE 4
#define DRONES_CAMARA 1
#define DRONES_POR_ENJAMBRE (DRONES_ATAQUE + DRONES_CAMARA)


// Función para mostrar los drones de cada enjambre con jerarquía y tipo
void mostrar_drones_por_enjambre(int enjambre_id, int base_drone_id) {
    printf("│\n");
    printf("├── Camión %d\n", enjambre_id+1);
    printf("|      |----- Enjambre %d (proceso)\n", enjambre_id+1);
    for (int i = 0; i < DRONES_ATAQUE; i++) {
        printf("|      |         ├── Drone %d [Ataque] (proceso + hilos nav/comb/combustible/ataque)\n", base_drone_id + i);
    }
    printf("|      |         └── Drone %d [Cámara] (proceso + hilos nav/comb/combustible/camara)\n", base_drone_id + DRONES_ATAQUE);
    printf("│\n");
}
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/wait.h>

#define SIZE 100

#define NUM_CAMIONES 3
#define NUM_OBJETIVOS 3
#define NUM_ENJAMBRES 3

// Hilos para drones de ataque y de cámara
#define HILOS_COMUNES 3 // nav, comb, combustible
#define HILO_ATAQUE 1
#define HILO_CAMARA 1

// Estructuras para posiciones
struct Posicion {
    int x;
    int y;
};


// Posiciones iniciales actualizadas
struct Posicion camiones[NUM_CAMIONES] = {
    {25, 0}, // Camion 1
    {50, 0}, // Camion 2
    {75, 0}  // Camion 3
};

struct Posicion objetivos[NUM_OBJETIVOS] = {
    {0, 25}, // Objetivo 1 (puedes ajustar si lo deseas)
    {0, 50}, // Objetivo 2
    {0, 75}  // Objetivo 3
};

struct Posicion puesto_control = {100, 0};

// Torretas
#define NUM_TORRETAS 2
struct Posicion torretas[NUM_TORRETAS] = {
    {10, 100},
    {90, 100}
};

// Zonas
struct Zona {
    int y_inicio;
    int y_fin;
    char nombre[32];
};

#define NUM_ZONAS 3
struct Zona zonas[NUM_ZONAS] = {
    {0, 33, "Zona de Ensamblaje"},
    {33, 66, "Zona de Defensa"},
    {66, 100, "Zona de Reensamblaje"}
};



// Hilos comunes
void* hilo_nav(void* arg) {
    int drone_id = *(int*)arg;
    printf("[Drone %d] Hilo de navegación activo.\n", drone_id);
    sleep(1);
    pthread_exit(NULL);
}
void* hilo_comb(void* arg) {
    int drone_id = *(int*)arg;
    printf("[Drone %d] Hilo de comunicación activo.\n", drone_id);
    sleep(1);
    pthread_exit(NULL);
}
void* hilo_combustible(void* arg) {
    int drone_id = *(int*)arg;
    printf("[Drone %d] Hilo de combustible activo.\n", drone_id);
    sleep(1);
    pthread_exit(NULL);
}
// Hilo específico de ataque
void* hilo_ataque(void* arg) {
    int drone_id = *(int*)arg;
    printf("[Drone %d] Hilo de ataque activo.\n", drone_id);
    sleep(1);
    pthread_exit(NULL);
}
// Hilo específico de cámara
void* hilo_camara(void* arg) {
    int drone_id = *(int*)arg;
    printf("[Drone %d] Hilo de cámara activo.\n", drone_id);
    sleep(1);
    pthread_exit(NULL);
}

// Proceso drone
void proceso_drone(int drone_id, int tipo) {
    pthread_t hilos[HILOS_COMUNES + 1];
    pthread_create(&hilos[0], NULL, hilo_nav, &drone_id);
    pthread_create(&hilos[1], NULL, hilo_comb, &drone_id);
    pthread_create(&hilos[2], NULL, hilo_combustible, &drone_id);
    if (tipo == 0) { // Ataque
        pthread_create(&hilos[3], NULL, hilo_ataque, &drone_id);
    } else { // Cámara
        pthread_create(&hilos[3], NULL, hilo_camara, &drone_id);
    }
    for (int i = 0; i < HILOS_COMUNES + 1; i++) {
        pthread_join(hilos[i], NULL);
    }
    // Enviar mensaje al centro de control por FIFO global
    int fd = open(FIFO_CENTRO, O_WRONLY);
    if (fd >= 0) {
        char msg[128];
        if (tipo == 0)
            snprintf(msg, sizeof(msg), "[Drone %d] (Ataque) Todos los hilos finalizaron.\n", drone_id);
        else
            snprintf(msg, sizeof(msg), "[Drone %d] (Cámara) Todos los hilos finalizaron.\n", drone_id);
        write(fd, msg, strlen(msg));
        close(fd);
    }
    exit(0);
}


// Hilo para leer mensajes de los drones desde el FIFO global
void* leer_fifo_centro(void* arg) {
    int fd = open(FIFO_CENTRO, O_RDONLY);
    if (fd < 0) {
        perror("[Centro de Control] Error abriendo FIFO_CENTRO para lectura");
        pthread_exit(NULL);
    }
    char buffer[256];
    while (1) {
        ssize_t n = read(fd, buffer, sizeof(buffer)-1);
        if (n > 0) {
            buffer[n] = '\0';
            printf("[Centro de Control] Mensaje recibido: %s\n", buffer);
        } else {
            usleep(100000); // Espera corta para evitar busy wait
        }
    }
    close(fd);
    pthread_exit(NULL);
}

// Proceso enjambre (camión)
void proceso_enjambre(int enjambre_id, int base_drone_id) {
    sem_t *sem_prev = NULL, *sem_next = NULL;
    // Sincronización: Enjambre 2 espera a 1, Enjambre 3 espera a 2
    if (enjambre_id == 1) {
        sem_prev = sem_open(SEM_ENJAMBRE1, O_CREAT, 0644, 0);
        sem_wait(sem_prev); // Espera a que el 1 termine
    } else if (enjambre_id == 2) {
        sem_prev = sem_open(SEM_ENJAMBRE2, O_CREAT, 0644, 0);
        sem_wait(sem_prev); // Espera a que el 2 termine
    }

    mostrar_drones_por_enjambre(enjambre_id, base_drone_id);


    pid_t drone_pids[DRONES_POR_ENJAMBRE];
    // Drones de ataque
    for (int i = 0; i < DRONES_ATAQUE; i++) {
        if ((drone_pids[i] = fork()) == 0) {
            proceso_drone(base_drone_id + i, 0); // 0 = ataque
        }
    }
    // Drone de cámara
    if ((drone_pids[DRONES_ATAQUE] = fork()) == 0) {
        proceso_drone(base_drone_id + DRONES_ATAQUE, 1); // 1 = camara
    }
    // Esperar a que terminen los drones
    for (int i = 0; i < DRONES_POR_ENJAMBRE; i++) {
        waitpid(drone_pids[i], NULL, 0);
    }
    printf("[Enjambre %d] Todos los drones finalizaron.\n", enjambre_id+1);

    // Señalizar al siguiente enjambre
    if (enjambre_id == 0) {
        sem_next = sem_open(SEM_ENJAMBRE1, O_CREAT, 0644, 0);
        sem_post(sem_next);
        sem_close(sem_next);
        sem_unlink(SEM_ENJAMBRE1);
    } else if (enjambre_id == 1) {
        sem_next = sem_open(SEM_ENJAMBRE2, O_CREAT, 0644, 0);
        sem_post(sem_next);
        sem_close(sem_next);
        sem_unlink(SEM_ENJAMBRE2);
    }
    if (sem_prev) {
        sem_close(sem_prev);
    }
    exit(0);
}

// Proceso camión
void proceso_camion(int camion_id) {
    printf("[Camion %d] Iniciado en posicion (%d,%d)\n", camion_id+1, camiones[camion_id].x, camiones[camion_id].y);
    // Simular trabajo
    sleep(3);
    exit(0);
}

// Proceso objetivo (puede ser artillería)
void proceso_objetivo(int objetivo_id) {
    printf("[Objetivo %d] Iniciado en posicion (%d,%d)\n", objetivo_id+1, objetivos[objetivo_id].x, objetivos[objetivo_id].y);
    // Simular trabajo
    sleep(3);
    exit(0);
}


int main() {
    pid_t enjambre_pids[NUM_ENJAMBRES];
    pid_t objetivo_pids[NUM_OBJETIVOS];
    pthread_t hilo_lector;

    // Mostrar configuración inicial
    printf("\n--- CONFIGURACIÓN DEL CAMPO DE BATALLA ---\n");
    printf("Centro de Comandos: (%d, %d)\n", puesto_control.x, puesto_control.y);
    for (int i = 0; i < NUM_CAMIONES; i++) {
        printf("Camión %d: (%d, %d)\n", i+1, camiones[i].x, camiones[i].y);
    }
    for (int i = 0; i < NUM_TORRETAS; i++) {
        printf("Torreta %d: (%d, %d)\n", i+1, torretas[i].x, torretas[i].y);
    }
    for (int i = 0; i < NUM_ZONAS; i++) {
        printf("%s: y = %d a %d\n", zonas[i].nombre, zonas[i].y_inicio, zonas[i].y_fin);
    }
    printf("------------------------------------------\n\n");

    // Crear FIFO global para comunicación de drones
    unlink(FIFO_CENTRO);
    if (mkfifo(FIFO_CENTRO, 0666) < 0) {
        perror("mkfifo centro");
    }

    // Lanzar procesos de objetivos (artillería)
    for (int i = 0; i < NUM_OBJETIVOS; i++) {
        if ((objetivo_pids[i] = fork()) == 0) {
            proceso_objetivo(i);
        }
    }

    // Lanzar procesos de enjambres/camiones
    int drone_id = 1;
    for (int i = 0; i < NUM_ENJAMBRES; i++) {
        if ((enjambre_pids[i] = fork()) == 0) {
            proceso_enjambre(i, drone_id);
        }
        drone_id += DRONES_POR_ENJAMBRE;
    }

    // Hilo para leer mensajes de los drones
    pthread_create(&hilo_lector, NULL, leer_fifo_centro, NULL);

    // Esperar a que terminen los enjambres
    for (int i = 0; i < NUM_ENJAMBRES; i++) {
        waitpid(enjambre_pids[i], NULL, 0);
    }
    // Esperar a que terminen los objetivos
    for (int i = 0; i < NUM_OBJETIVOS; i++) {
        waitpid(objetivo_pids[i], NULL, 0);
    }

    // Cancelar hilo de lectura
    pthread_cancel(hilo_lector);
    pthread_join(hilo_lector, NULL);

    // Eliminar FIFO global
    unlink(FIFO_CENTRO);

    printf("[Centro de Control] Todos los procesos han terminado.\n");
    return 0;
}
