# Laboratorio No. 04 – Robótica de Desarrollo, Intro a ROS 2 Humble – Turtlesim

## Integrates

- Sergio Andrés Bolaños Penagos
- Sergio Felipe Rodriguez Mayorga

## Resultados de aprendizaje

- Conocer y explicar los conceptos básicos de ROS (Robot Operating System).
- Usar los comandos fundamentales de Linux.
- Conectar nodos de ROS 2 con Python.

## Conceptos básicos de ROS 2 utilizados

En este laboratorio se aplican algunos de los conceptos fundamentales de ROS 2:

- **Nodos**  
  Son procesos que realizan una o varias tareas.  
  En este proyecto:
  - `turtlesim_node` es el nodo del simulador que dibuja y actualiza la tortuga en el canvas.
  - `move_turtle2.py` (`TurtleController`) es el nodo de control que envía comandos de velocidad, escucha la pose y gestiona el teclado.

- **Tópicos (topics)**  
  Canales de comunicación asíncrona para publicar y suscribirse a mensajes, como:
  - `/turtle1/cmd_vel` (`geometry_msgs/msg/Twist`) donde el nodo de control publica velocidades lineales y angulares para mover la tortuga.
  - `/turtle1/pose` (`turtlesim/msg/Pose`): donde `turtlesim_node` publica continuamente la posición y orientación actuales de la tortuga.

- **Servicios (services)**  
  Llamadas tipo petición–respuesta para operaciones puntuales:
  - `/reset` (`std_srvs/srv/Empty`): utilizado por `TurtleController` para limpiar el lienzo y reiniciar la simulación.

- **Paquetes (packages)**  
  Son las unidades que agrupan nodos, configuración del paquete y librerías y metadatos:
  - `my_turtle_controller` es el paquete de Python que contiene toda nuestra lógica: el nodo `move_turtle2.py`, el archivo `setup.py`, `package.xml`, tests (no utilizados) y recursos (no utilizaods).

- **Workspace**  
  Estructura de directorios donde se construyen y organizan varios paquetes con `colcon`.  
  - En este caso es `ros2_ws`, que incluye el paquete `my_turtle_controller` y se compila con `colcon build`.

Estos conceptos se combinan para lograr que el nodo `TurtleController` controle por completo el simulador `turtlesim` usando únicamente ROS 2, Python y los nodos, tópicos y servicios de TurtleSim.

## Descripción general del laboratorio

El laboratorio busca que los estudiantes se familiaricen con ROS, puedan explicar los conceptos básicos de ROS y lo apliquen en un problema de robótica móvil.

En este laboratorio se cuenta con un paquete llamado `my_turtle_controller`, cuyo nodo principal actualmente es `move_turtle2.py`. Este nodo:
- Se conecta al tópico `/turtle1/cmd_vel` para enviar comandos de velocidad.
- Se suscribe a `/turtle1/pose` para conocer la posición y orientación de la tortuga.
- Lee el teclado directamente desde la terminal (sin usar `turtle_teleop_key`).
- Implementa control manual con flechas y funciones automáticas para dibujar las letras **SABP** en la parte superior y **SFRM** en la parte inferior del mapa, además de letras individuales.

## Cómo correr el código

```bash
# Terminal 1: lanzar el simulador
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node

# Terminal 2: compilar y ejecutar el paquete
source /opt/ros/humble/setup.bash
cd .  # raíz de este repositorio
colcon build
source install/setup.bash
ros2 run my_turtle_controller move_turtle
```

## Procedimiento y desarrollo

Para realizar este laboratorio se partió del código básico de `move_turtle` dado en la clase de laboratorio y disponible en el repositorio [Introducción a TurtleSim](https://github.com/labsir-un/ROB_Intro_ROS2_Humble_Turtlesim) del laboratorio.

A partir de allí se hizo lo siguiente:

1. Instalar y lanzar turtlesim (en cada terminal: `source /opt/ros/humble/setup.bash`): `sudo apt update && sudo apt install ros-humble-turtlesim`, luego `ros2 run turtlesim turtlesim_node` y en otra terminal `ros2 run turtlesim turtle_teleop_key`.
2. Usar rqt y servicios: `sudo apt install '~nros-humble-rqt*'`, abrir `rqt`, llamar `/spawn` (turtle2 en x=1.0,y=1.0) y `/turtle1/set_pen` (r=255,g=0,b=0,width=5). Controlar turtle2 con remapeo: `ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel`.
3. Compilar y correr el nodo Python del paquete: en el workspace `colcon build && source install/setup.bash` y ejecutar `ros2 run my_turtle_controller move_turtle`.

Como parte de este laboratorio se solicitaba que se pudiera controlar la tortuga tanto con las flechas del teclado como de manera automática para las iniciales de todos los nombres y apellidos de los integrantes del grupo, para ello, se modificó el nodo `TurtleController` de `move_turtle2.py` con las siguientes funciones:


### 1. Publicación y recepción de mensajes

Publica mensajes `Twist` en `/turtle1/cmd_vel`.
- Se suscribe a `turtlesim/msg/Pose` en `/turtle1/pose` para tener realimentación de posición y orientación.

### 2. Reinicio de la simulación

- Crea un cliente para el servicio `/reset` (`std_srvs/srv/Empty`) para limpiar la pantalla y reiniciar la simulación.

### 3. Lectura de teclado
- Se usa la entrada estándar (`sys.stdin`) con `termios` y `tty` para poner la terminal en modo raw y leer teclas individuales sin necesidad de presionar Enter.
- La función `get_key()` devuelve:
  - Secuencias de escape para flechas (`\x1b[A`, `\x1b[B`, `\x1b[C`, `\x1b[D`).
  - Caracteres individuales (`'1'`, `'2'`, `'3'`, `'c'`, `'h'`, `'s'`, `'a'`, etc.).

### **4. Funcionamiento principal**  
- En `main()` se instancia `TurtleController` y se entra en un bucle donde:
  - Se lee una tecla.
  - Se decide la acción (control manual, dibujo de una secuencia, dibujo de una letra, limpiar, ir al origen o salir).

### 5. **Decisiones de diseño clave**
1. **Movimiento continuo de letras**:  
  Las letras se dibujan con funciones `*_continuous` que asumen un punto de partida y genera un trazo que se aproxima a uno continuo, con la excepción de los casos en donde cambia de orientación, como al pasar de una recta a un semicirculo.
2. **Evitar tachar el dibujo**:  
  Para ir a los puntos de inicio y al “home”, se usa `move_to_start_L(...)`, que primero ajusta la coordenada X y luego la Y, produciendo un movimiento en “L” que evita cruzar las letras dibujadas.
3. **Movimiento por medio de comandos de velocidad, ángulos y tiempos de duración**
  Para realizar los movimientos la tortuga utiliza 2 funciones principalmente, `move_timed` y `rotate_to_angle`, las cuales se explican con mayor detalle más adelante.

---

### Control manual de movimiento con flechas  

En `move_turtle2.py` se leen las teclas para movimiento manual de la tortuga, a través de la función `get_key()`.
- `'\x1b[A'` (flecha ↑): `node.send_cmd(2.0, 0.0)` → avance lineal hacia adelante.
- `'\x1b[B'` (flecha ↓): `node.send_cmd(-2.0, 0.0)` → movimiento hacia atrás.
- `'\x1b[D'` (flecha ←): `node.send_cmd(0.0, 2.0)` → giro en el lugar a la izquierda.
- `'\x1b[C'` (flecha →): `node.send_cmd(0.0, -2.0)` → giro en el lugar a la derecha.


Cuando no se detecta ninguna tecla, se llama a `node.stop()` para publicar un `Twist` nulo y detener la tortuga.

---

### Dibujo automático de letras personalizadas (SABP y SFRM)  
  
Se diseñó una solución extendida que dibuja dos palabras con trazo continuo, en la parte superior se dibuja `SABP` con una línea que conecta la parte superior de las letras, mientras que en la parte inferior se dibbuja `SFRM` utilizando el mismo criterio.

Para ello se utilizan las siguientes funciones:
- `draw_S_continuous(self)`
- `draw_A_continuous(self)`
- `draw_B_continuous(self)`  
- `draw_P_continuous(self)`
- `draw_F_continuous(self)`
- `draw_R_continuous(self)`
- `draw_M_continuous(self)`
Cada una de ellas asume una posición y orientación inicial, y utiliza combinaciones de `move_timed(linear, angular, duration)` y `rotate_to_angle(...)` para trazar segmentos rectos, curvas y diagonales que aproximan la forma de la letra.

Estos métodos están calibrados para respetar una altura máxima y un ancho específico, facilitando que todas las letras queden alineadas en una cuadrícula.

En la terminal se cuenta con las siguientes funciones:
#### `draw_SABP(self)`:
Lleva primero a la tortuga a la parte superior izquierda con `move_to_start_L(1.0, 6.0)`.
Llama en cadena a `draw_S_continuous()`, `draw_A_continuous()`, `draw_B_continuous()`, `draw_P_continuous()`, insertando pequeños desplazamientos horizontales entre letras.
Finaliza volviendo al centro del mapa (`move_to_start_L(5.5, 5.5)`).
#### `draw_SFRM(self)`:
Lleva a la tortuga a la parte inferior izquierda con `move_to_start_L(1.0, 1.0)`.
Ejecuta `draw_S_continuous()`, `draw_F_continuous()`, `draw_R_continuous()`, `draw_M_continuous()` con espaciado horizontal.
Sube ligeramente y luego usa `move_to_start_L(5.5, 5.5)` para volver al centro evitando tachar.
#### `draw_FULL(self)`:
Ejecuta `draw_SABP()` y, a continuación, `draw_SFRM()` en una sola rutina.

Para realizar el dibujo automático se cuenta con unas teclados predefinidas que ejecutan los métodos de dibujo completo y por letras.
- `'1'`: dibuja la secuencia **SABP** (`node.draw_SABP()`).
- `'2'`: dibuja la secuencia **SFRM** (`node.draw_SFRM()`).
- `'3'`: ejecuta la rutina completa **SABP + SFRM** (`node.draw_FULL()`).
- Letras individuales:
  - `'s'` / `'S'`: `draw_S_continuous()`
  - `'a'` / `'A'`: `draw_A_continuous()`
  - `'b'` / `'B'`: `draw_B_continuous()`
  - `'p'` / `'P'`: `draw_P_continuous()`
  - `'f'` / `'F'`: `draw_F_continuous()`
  - `'r'` / `'R'`: `draw_R_continuous()`
  - `'m'` / `'M'`: `draw_M_continuous()`


### Comandos adicionales: limpiar trazo y reiniciar  

#### Servicio `/reset`**:
  - Se crea un cliente `reset_client = self.create_client(Empty, 'reset')` en el constructor.
  - La función `reset_simulation(self)` envía una petición `Empty` al servicio `/reset`, que:
    - Borra todos los trazos.
    - Reinicia la posición y orientación de la tortuga a los valores por defecto.
#### Teclas de ayuda para el canvas
  - `'c'`: llama a `reset_simulation()` (limpia y reinicia el simulador).
  - `'h'`: llama a `move_to_start_L(5.54, 5.54)` para mover la tortuga de vuelta al centro del mapa sin borrar el trazo.


### Diagrama de flujo de la solución (Mermaid)
El siguiente diagrama resume el comportamiento principal del nodo `move_turtle2.py`:

 ```mermaid
  %%{init: { "theme": "", "flowchart": { "nodeSpacing": 3, "rankSpacing": 100 }, "fontSize":0} }%%
flowchart TD
    A["Inicio nodo TurtleController"] --> B["Suscribirse a /turtle1/pose<br/>Crear publisher /turtle1/cmd_vel<br/>Crear cliente /reset"]
    B --> C["Esperar tecla en bucle"]

    C -->|"Flechas ↑↓←→"| D["Control manual<br/>send_cmd(linear, angular)"]
    C -->|"1"| E["SABP<br/>draw_SABP()"]
    C -->|"2"| F["SFRM<br/>draw_SFRM()"]
    C -->|"3"| G["Secuencia completa<br/>draw_FULL()"]
    C -->|"s,a,b,p,f,r,m"| H["Dibujo de letra individual<br/>funciones *_continuous"]
    C -->|"c"| I["Llamar reset_simulation()<br/>Servicio /reset"]
    C -->|"h"| J["Ir al origen<br/>move_to_start_L(5.54, 5.54)"]
    C -->|"q"| K["Salir<br/>stop() y shutdown()"]

    D --> C
    E --> C
    F --> C
    G --> C
    H --> C
    I --> C
    J --> C

```

### Organización del código fuente

Con el fin de mantener un código fuente profesional se dividió de la siguiente manera:
- `src/my_turtle_controller/`
  - `my_turtle_controller/`
    - `__init__.py`
    - `move_turtle2.py`  
      Nodo principal con:
      - Clase `TurtleController` (publicador, suscriptor, cliente de servicio).
      - Funciones de ayuda de movimiento (`move_timed`, `go_to_point`, `move_to_start_L`, `reset_pen_pos`).
      - Funciones de dibujo de letras simples (`draw_S`, `draw_A`, etc.) y continuas (`draw_S_continuous`, etc.).
      - Rutinas de alto nivel (`draw_SABP`, `draw_SFRM`, `draw_FULL`).
      - Bucle principal `main()` con lectura de teclado.
  - `setup.py`  
    - Declara el paquete `my_turtle_controller` y el entry point:
      - `move_turtle = my_turtle_controller.move_turtle2:main`
  - `package.xml`  
    - Metadatos del paquete y dependencias de `rclpy`, `geometry_msgs`, `turtlesim`, `std_srvs`, etc.
- `README.md`
  - Documentación del laboratorio, diagrama Mermaid y descripción del código.

---
### Descripción de las funciones internas de movimiento (`move_turtle2.py`)

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


### Resultados

Como resultado de la práctica de laboratorio de ROS con Turtlesim, se presenta el siguiente video donde se explica cómo correr el código y la demostración de las funciones principales con las que cuenta move_turtle2:

<video width="1080" height="720" controls>
  <source src="https://drive.google.com/file/d/1A1xO4MQUV6Bx2qG8Lt7cUjtGW7hwVwmP/view?usp=sharing" type="video/mp4">
  Tu navegador no soporta video HTML5.
</video>

[Aquí se puede ver el video de la simulacion de turtlesim con el controlador move_turtle2:](https://drive.google.com/file/d/1A1xO4MQUV6Bx2qG8Lt7cUjtGW7hwVwmP/view?usp=sharing)



### Conclusiones y aprendizajes

1. Se logró implementar un nodo ROS 2 en Python que controla completamente `turtlesim` desde el teclado, respetando la restricción de no usar `turtle_teleop_key`.  
2. Se diseñó un sistema de dibujo de letras continuas que mantiene tamaños y espaciados coherentes, permitiendo escribir palabras completas (**SABP** y **SFRM**) y letras individuales. Pese a esto, se tuvo problemas para dibujar las letras de manera precisa. 
3. El uso de `turtlesim/msg/Pose` permitió incorporar control en lazo cerrado (por ejemplo, para ir a posiciones específicas y evitar cruces no deseados), mejorando la precisión algunos de los trazos.  
4. La integración del servicio `/reset` y de comandos de “home” facilita la experimentación rápida, algo muy útil en un entorno de laboratorio.  
5. En conjunto, el laboratorio refuerza el entendimiento de nodos, tópicos, servicios y la interacción entre ROS 2 y Python.
