# 🚁 Drone Wars 2 - Instrucciones de Compilación

## 🔨 Compilar el Código

### Comando de Compilación:
```bash
gcc -o drone_wars2 drone_wars2.c -lpthread -lm
```

### Librerías Necesarias:
- **`-lpthread`** - Para soporte de hilos (pthreads)
- **`-lm`** - Para funciones matemáticas (math.h)

## 🚀 Ejecutar la Simulación:

```bash
# Compilar
gcc -o drone_wars2 drone_wars2.c -lpthread -lm

# Ejecutar
./drone_wars2 config.txt

## 📊 Características de Distancia:

La simulación ahora muestra **información detallada de distancia** en tiempo real:

- **Progreso de distancia** cada 10-15 unidades durante el vuelo
- **Posición actual** de cada drone en coordenadas (X,Y)
- **Estadísticas por enjambre**: total, promedio, máximo y mínimo
- **Resumen final** de distancias totales del sistema
- **Distancia al objetivo** durante el ataque
- **Patrulla circular** con seguimiento de distancia

## 🎯 Información Mostrada:

- **Estado de cada drone** con distancia recorrida y posición
- **Estadísticas por enjambre** con métricas de distancia
- **Progreso en tiempo real** durante la navegación
- **Resumen final** completo del sistema