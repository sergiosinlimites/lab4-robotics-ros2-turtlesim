## Descripción de las funciones internas de movimiento (`move_turtle2.py`)

Este documento explica **únicamente** las funciones internas de `TurtleController` que se encargan de:
- Mantener el estado de la tortuga.
- Enviar comandos a `turtlesim`.
- Calcular distancias, ángulos y movimientos básicos.

Las funciones de dibujo de letras (`draw_*` y `draw_*_continuous`) no se detallan aquí; ellas solo combinan estas primitivas de movimiento para formar las letras.

---

### Suscripción y estado interno

#### `update_pose(self, data: Pose)`
- Se llama cada vez que llega un mensaje al tópico `/turtle1/pose`.
- Actualiza la pose interna de la tortuga en `self.pose`:
  - `self.pose.x`: posición en el eje X.
  - `self.pose.y`: posición en el eje Y.
  - `self.pose.theta`: orientación en radianes.
- Esta información es la base para cualquier control en lazo cerrado (saber dónde está y hacia dónde mira la tortuga).

---

### Envío básico de comandos

#### `send_cmd(self, linear: float, angular: float)`
- Publica un mensaje `Twist` en `/turtle1/cmd_vel` con:
  - `linear.x = linear` (velocidad lineal en m/s).
  - `angular.z = angular` (velocidad angular en rad/s).
- Se utiliza principalmente para el **control manual** con las flechas del teclado.

#### `stop(self)`
- Envía un `Twist` con velocidades lineales y angulares cero.
- Se usa para detener la tortuga cuando:
  - No hay teclas activas.
  - Se termina un movimiento planificado (segmento de letra, arco, etc.).

---

### Funciones matemáticas de apoyo

#### `get_distance(self, goal_x: float, goal_y: float) -> float`
- Devuelve la distancia euclídea entre la posición actual `(x, y)` y un objetivo `(goal_x, goal_y)`:
  \[
  d = \sqrt{(goal\_x - x)^2 + (goal\_y - y)^2}
  \]
- Se usa en movimientos hacia un punto (`go_to_point`) para saber cuándo detenerse.

#### `normalize_angle(self, angle: float) -> float`
- Normaliza cualquier ángulo para que quede en el rango \([-π, π]\).
- Esto evita saltos bruscos al comparar orientaciones que cruzan el límite ±π (por ejemplo, de +3.14 a -3.14).

---

### Control de orientación y navegación

#### `rotate_to_angle(self, target_angle_rad: float, tolerance: float = 0.01)`
- Hace girar la tortuga hasta que su orientación `theta` esté próxima al ángulo deseado:
  - Calcula el error:  
    \[
    angle\_diff = normalize\_angle(target\_angle\_rad - \theta\_{actual})
    \]
  - Si el error es grande, usa una velocidad angular alta (±2.0 rad/s).
  - Cuando el error es pequeño, reduce la velocidad (±0.5 rad/s) para no sobrepasar el ángulo objetivo.
  - Se detiene cuando \(|angle\_diff| < tolerance\).

#### `go_to_point(self, goal_x: float, goal_y: float, tolerance: float = 0.1)`
- Implementa un control proporcional sencillo para ir a un punto \((goal_x, goal_y)\):
  1. Calcula el ángulo deseado:
     \[
     \theta\_{des} = \text{atan2}(goal\_y - y,\; goal\_x - x)
     \]
  2. Calcula el error angular:
     \[
     angle\_diff = normalize\_angle(\theta\_{des} - \theta\_{actual})
     \]
  3. Estrategia de movimiento:
     - Si \(|angle\_diff|\) es grande → solo gira en el sitio (modifica `angular.z`) hasta alinearse.
     - Si \(|angle\_diff|\) es pequeño → avanza hacia el objetivo:
       - `linear.x` proporcional a la distancia restante (limitado a una velocidad máxima).
       - `angular.z` proporcional al error angular (corrige el rumbo).
  4. Termina cuando `get_distance(goal_x, goal_y) <= tolerance`.

---

### Movimiento elemental para las letras

#### `move_timed(self, linear: float, angular: float, duration: float)`
Es la primitiva que usan las funciones `draw_*_continuous` para construir líneas rectas, giros y arcos:

- Publica un `Twist` constante con:
  - `linear.x = linear`
  - `angular.z = angular`
  durante `duration` segundos.
- Dentro del bucle:
  - Se publica el comando de velocidad en `/turtle1/cmd_vel`.
  - Se llama a `rclpy.spin_once(self, timeout_sec=0.0)` para mantener `self.pose` actualizada.
  - Se duerme un pequeño intervalo (`time.sleep(0.02)`), obteniendo un control en tiempo discreto.
- Al final se llama a `stop()` para garantizar que la tortuga quede detenida.

Aunque `move_timed` trabaja con **velocidad + tiempo**, los valores de `linear`, `angular` y `duration` han sido ajustados para que la **distancia y el ángulo recorridos sean muy reproducibles**. De esta forma, las letras SABP y SFRM se dibujan de manera consistente en el mismo lugar del mapa.

---

### Movimientos compuestos y reinicio

#### `move_to_start_L(self, target_x: float, target_y: float)`
- Mueve la tortuga hasta un punto objetivo siguiendo un patrón en “L”:
  1. `go_to_point(target_x, self.pose.y)` → ajusta primero la coordenada **X**.
  2. `go_to_point(target_x, target_y)` → ajusta después la coordenada **Y**.
  3. `rotate_to_angle(0.0)` → orienta la tortuga hacia la derecha.
- Se usa para ir a:
  - Puntos de inicio de las palabras (esquina superior o inferior).
  - El “home” en el centro del mapa (5.5, 5.5).

#### `reset_pen_pos(self, x: float, y: float)`
- Es una función de conveniencia que simplemente llama a `move_to_start_L(x, y)`.
- Se utiliza en algunas rutinas para reposicionar la tortuga antes de empezar un nuevo trazo.

#### `reset_simulation(self)`
- Cliente del servicio `/reset` de `turtlesim` (`std_srvs/srv/Empty`):
  - Verifica que el servicio está disponible con `wait_for_service`.
  - Envía una petición `Empty.Request()` usando `call_async`.
  - El efecto en `turtlesim` es:
    - Limpiar todo el trazo dibujado.
    - Devolver la tortuga a la posición y orientación iniciales.

Estas funciones forman el “motor” matemático y de control de `move_turtle2.py`.  

