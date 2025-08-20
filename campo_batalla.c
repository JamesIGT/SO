
// Incluir stdio.h primero para evitar warnings de printf
#include <stdio.h>
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

// Posiciones iniciales
struct Posicion camiones[NUM_CAMIONES] = {
    {0, 25}, // Camion 1
    {0, 75}, // Camion 2
    {0, 50}  // Camion 3
};

struct Posicion objetivos[NUM_OBJETIVOS] = {
    {0, 25}, // Objetivo 1
    {0, 50}, // Objetivo 2
    {0, 75}  // Objetivo 3
};

struct Posicion puesto_control = {0, 100};



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
    if (tipo == 0)
        printf("[Drone %d] (Ataque) Todos los hilos finalizaron.\n", drone_id);
    else
        printf("[Drone %d] (Cámara) Todos los hilos finalizaron.\n", drone_id);
    exit(0);
}


// Hilo para mostrar estado de todos los drones
void* mostrar_estado(void* arg) {
    while (1) {
        printf("[Centro de Control] Mostrando estado de todos los drones...\n");
        sleep(2);
    }
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
    pthread_t hilo_estado;

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

    // Hilo para mostrar estado de todos los drones
    pthread_create(&hilo_estado, NULL, mostrar_estado, NULL);

    // Esperar a que terminen los enjambres
    for (int i = 0; i < NUM_ENJAMBRES; i++) {
        waitpid(enjambre_pids[i], NULL, 0);
    }
    // Esperar a que terminen los objetivos
    for (int i = 0; i < NUM_OBJETIVOS; i++) {
        waitpid(objetivo_pids[i], NULL, 0);
    }

    // Cancelar hilo de estado
    pthread_cancel(hilo_estado);
    pthread_join(hilo_estado, NULL);

    printf("[Centro de Control] Todos los procesos han terminado.\n");
    return 0;
}
