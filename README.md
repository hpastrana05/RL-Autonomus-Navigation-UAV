# RL Autonomus Navigation UAV
El proyecto trata de usar un entorno virtual de control de drones, y una vez tener el entorno bien configurado, usar datos via imagenes/sensores que ve el dron para mediante Reinforce Learning hacer que vaya del punto A al punto B esquivando obstaculos.

---
## Para correr archivos

Hay que clonar en la misma carpeta padre de este repositorio, el repositorio de `gym-pybullet-drones`

```
git clone https://github.com/learnsyslab/gym-pybullet-drones.git
```

Y para correr los archivos que se crean, desde la terminal con el directorio root de este repositorio abierto hay que correr el siguiente comando.

```
python [archivo.py] --plot false --duration_sec [tiempo]
```

---
## Entorno

1. **Gym pybullet drones:**   [link](https://learnsyslab.github.io/gym-pybullet-drones/) Entorno virtual para controlar uno o mas drones con datos en tiempo real.

---
### TODO

- [ ] Usar sensores para evitar paredes
- [ ] Establecer las observaciones para el RL