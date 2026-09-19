# RL Autonomus Navigation UAV
El proyecto trata de usar un entorno virtual de control de drones, y una vez tener el entorno bien configurado, usar datos via imagenes/sensores que ve el dron para mediante Reinforce Learning hacer que vaya del punto A al punto B esquivando obstaculos.

---
## Para correr archivos

```
../gym-pybullet-drones/.venv/bin/python [archivo.py] --plot false --duration_sec [tiempo]
```

---
## Entorno

1. **Gym pybullet drones:**   [link](https://learnsyslab.github.io/gym-pybullet-drones/) Entorno virtual para controlar uno o mas drones con datos en tiempo real.

---
## Baseline

El Dron tiene que recibir la acción de ir de un punto A a un punto B y tiene que encontrar la ruta mas optima y siendo capaz de esquivar obstaculos como personas, arboles, edificios, paredes, etc.

---
## Ideas

1. Se pueden tener distintas listas de acciones para modelos distintos:
	- Una lista de acción que contenga movimientos Arriba, Abajo, Giro izq, Giro der, etc.
	- Otra lista que contenga las posibles acciones de los motores 
	