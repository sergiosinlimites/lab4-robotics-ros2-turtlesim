## Laboratorio No. 04 – Robótica de Desarrollo, Intro a ROS 2 Humble – Turtlesim

### Descripción general del laboratorio  
**Requerimiento del laboratorio**  
Desarrollar un nodo en ROS 2 Humble que controle la tortuga del simulador `turtlesim` exclusivamente desde un script en Python (`move_turtle.py`), permitiendo control manual con el teclado y el dibujo automático de letras personalizadas, según el enunciado del laboratorio [`Laboratorio_No__04___2025_II___Robótica_de_Desarrollo__Intro_a_ROS__Copy_.pdf`](file:///home/sergio/ros2_ws/Laboratorio_No__04___2025_II___Robo%CC%81tica_de_Desarrollo__Intro_a_ROS__Copy_.pdf).

**Implementación en este repositorio**  
El workspace `ros2_ws` contiene el paquete `my_turtle_controller`, cuyo nodo principal actualmente es `move_turtle2.py`. Este nodo:
- Se conecta al tópico `/turtle1/cmd_vel` para enviar comandos de velocidad.
- Se suscribe a `/turtle1/pose` para conocer la posición y orientación de la tortuga.
- Lee el teclado directamente desde la terminal (sin usar `turtle_teleop_key`).
- Implementa control manual con flechas y funciones automáticas para dibujar las letras **SABP** en la parte superior y **SFRM** en la parte inferior del mapa, además de letras individuales.

---

### Objetivos y resultados de aprendizaje  
**Requerimiento del laboratorio**  
Según el enunciado, los resultados de aprendizaje son:
- Conocer y explicar los conceptos básicos de ROS (Robot Operating System).
- Usar los comandos fundamentales de Linux.
- Conectar nodos de ROS 2 con Python.

**Implementación en este repositorio**  
- **Conceptos básicos de ROS 2**:  
  - Se utilizan nodos (`TurtleController`), tópicos (`/turtle1/cmd_vel`, `/turtle1/pose`) y servicios (`/reset`) para controlar el simulador.
- **Comandos de Linux**:  
  - El flujo típico de uso involucra comandos como `colcon build`, `source install/setup.bash` y `ros2 run my_turtle_controller move_turtle`.
- **Nodos de ROS 2 en Python**:  
  - El archivo `src/my_turtle_controller/my_turtle_controller/move_turtle2.py` define un nodo ROS 2 en Python que encapsula todo el comportamiento solicitado (lectura de teclado, publicación de velocidades, suscripción a la pose y llamada a servicios).

---

### Procedimiento y desarrollo  
**Requerimiento del laboratorio**  
El README debe incluir una descripción detallada del desarrollo: objetivos, procedimientos, decisiones de diseño y funcionamiento general del proyecto.

**Implementación en este repositorio**  
1. **Creación del workspace y paquete**  
   - Se trabaja en el workspace `ros2_ws`.  
   - Dentro de `src/` se creó el paquete `my_turtle_controller` con soporte para nodos en Python (`setup.py`, `package.xml`).

2. **Nodo de control `TurtleController`**  
   - Clase principal definida en `move_turtle2.py` que:
     - Publica mensajes `Twist` en `/turtle1/cmd_vel`.
     - Se suscribe a `turtlesim/msg/Pose` en `/turtle1/pose` para tener realimentación de posición y orientación.
     - Crea un cliente para el servicio `/reset` (`std_srvs/srv/Empty`) para limpiar la pantalla y reiniciar la simulación.

3. **Lectura de teclado**  
   - Se usa la entrada estándar (`sys.stdin`) con `termios` y `tty` para poner la terminal en modo raw y leer teclas individuales sin necesidad de presionar Enter.
   - La función `get_key()` devuelve:
     - Secuencias de escape para flechas (`\x1b[A`, `\x1b[B`, `\x1b[C`, `\x1b[D`).
     - Caracteres individuales (`'1'`, `'2'`, `'3'`, `'c'`, `'h'`, `'s'`, `'a'`, etc.).

4. **Bucle principal del nodo**  
   - En `main()` se instancia `TurtleController` y se entra en un bucle donde:
     - Se lee una tecla.
     - Se decide la acción (control manual, dibujo de una secuencia, dibujo de una letra, limpiar, ir al origen o salir).
     - Se llama a `rclpy.spin_once` con `timeout` pequeño para mantener el nodo reactivo y actualizar la pose.

5. **Decisiones de diseño clave**  
   - **Control desde un solo script**:  
     Todo el control (manual y automático) se implementa en `move_turtle2.py`, respetando la restricción del laboratorio de no usar `turtle_teleop_key`.
   - **Movimiento continuo de letras**:  
     Las letras se dibujan con funciones `*_continuous` que asumen un punto de partida y generan un trazo continuo, cuidando la altura máxima (~4 unidades) y el ancho (~2–2.5 unidades) para mantener una cuadrícula visual ordenada.
   - **Evitar tachar el dibujo**:  
     Para ir a los puntos de inicio y al “home”, se usa `move_to_start_L(...)`, que primero ajusta la coordenada X y luego la Y, produciendo un movimiento en “L” que evita cruzar las letras dibujadas.

---

### Control manual de movimiento con flechas  
**Requerimiento del laboratorio**  
- Permitir mover la tortuga de forma lineal y angular utilizando las flechas del teclado:  
  - Flecha ↑: avanzar hacia adelante.  
  - Flecha ↓: retroceder.  
  - Flecha ←: girar a la izquierda.  
  - Flecha →: girar a la derecha.

**Implementación en este repositorio**  
En `move_turtle2.py`:
- La función `get_key()` lee la tecla actual de la terminal.
- En el bucle principal (`main()`), se interpretan las flechas:
  - `'\x1b[A'` (flecha ↑): `node.send_cmd(2.0, 0.0)` → avance lineal hacia adelante.
  - `'\x1b[B'` (flecha ↓): `node.send_cmd(-2.0, 0.0)` → movimiento hacia atrás.
  - `'\x1b[D'` (flecha ←): `node.send_cmd(0.0, 2.0)` → giro en el lugar a la izquierda.
  - `'\x1b[C'` (flecha →): `node.send_cmd(0.0, -2.0)` → giro en el lugar a la derecha.
- Cuando no se detecta ninguna tecla, se llama a `node.stop()` para publicar un `Twist` nulo y detener la tortuga.

---

### Dibujo automático de letras personalizadas (SABP y SFRM)  
**Requerimiento del laboratorio**  
- A partir de las iniciales de los nombres y apellidos del equipo, implementar funciones para dibujar letras con la tortuga.  
- Deben existir teclas que disparen dichas letras desde el script de control.

**Implementación en este repositorio**  
Se diseñó una solución extendida que dibuja dos palabras con trazo continuo:
- **Línea superior**: `SABP`.  
- **Línea inferior**: `SFRM`.

1. **Funciones de letras continuas**  
   - `draw_S_continuous(self)`  
   - `draw_A_continuous(self)`  
   - `draw_B_continuous(self)`  
   - `draw_P_continuous(self)`  
   - `draw_F_continuous(self)`  
   - `draw_R_continuous(self)`  
   - `draw_M_continuous(self)`  
   Cada una:
   - Asume una posición y orientación inicial estándar (por ejemplo, “debajo” de la letra previa).  
   - Usa combinaciones de `move_timed(linear, angular, duration)` y `rotate_to_angle(...)` para trazar segmentos rectos, curvas y diagonales que aproximan la forma de la letra.  
   - Está calibrada para respetar una altura máxima (~4 unidades) y un ancho específico, facilitando que todas las letras queden alineadas en una “cuadrícula”.

2. **Secuencias de palabras**  
   - `draw_SABP(self)`:
     - Lleva primero a la tortuga a la parte superior izquierda con `move_to_start_L(1.0, 6.0)`.
     - Llama en cadena a `draw_S_continuous()`, `draw_A_continuous()`, `draw_B_continuous()`, `draw_P_continuous()`, insertando pequeños desplazamientos horizontales entre letras.
     - Finaliza volviendo al centro del mapa (`move_to_start_L(5.5, 5.5)`).
   - `draw_SFRM(self)`:
     - Lleva a la tortuga a la parte inferior izquierda con `move_to_start_L(1.0, 1.0)`.
     - Ejecuta `draw_S_continuous()`, `draw_F_continuous()`, `draw_R_continuous()`, `draw_M_continuous()` con espaciado horizontal.
     - Sube ligeramente y luego usa `move_to_start_L(5.5, 5.5)` para volver al centro evitando tachar.
   - `draw_FULL(self)`:
     - Ejecuta `draw_SABP()` y, a continuación, `draw_SFRM()` en una sola rutina.

3. **Teclas asignadas en el nodo**  
En el `main()`:
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

Esto permite tanto demostraciones automáticas como escribir letras manualmente moviendo la tortuga con flechas y luego disparando cada letra individual.

---

### Comandos adicionales: limpiar trazo y reiniciar  
**Requerimiento interpretado**  
Aunque el PDF no lo exige explícitamente, para facilitar las pruebas se desea:
- Limpiar todo el trazo del `turtlesim`.
- Mover la tortuga al origen.
- Reiniciar la escena desde el nodo de control, sin editar el nodo del simulador.

**Implementación en este repositorio**  
- **Servicio `/reset`**:
  - Se crea un cliente `reset_client = self.create_client(Empty, 'reset')` en el constructor.
  - La función `reset_simulation(self)` envía una petición `Empty` al servicio `/reset`, que:
    - Borra todos los trazos.
    - Reinicia la posición y orientación de la tortuga a los valores por defecto.
- **Teclas asociadas**:
  - `'c'`: llama a `reset_simulation()` (limpia y reinicia el simulador).
  - `'h'`: llama a `move_to_start_L(5.54, 5.54)` para mover la tortuga de vuelta al centro del mapa sin borrar el trazo.

---

### Diagrama de flujo de la solución (Mermaid)  
**Requerimiento del laboratorio**  
Incluir un diagrama de flujo implementado en Mermaid que represente claramente el funcionamiento de la solución.

**Implementación en este repositorio**  
El siguiente diagrama resume el comportamiento principal del nodo `move_turtle2.py`:

```mermaid
flowchart TD
    A[Inicio nodo TurtleController] --> B[Suscribirse a /turtle1/pose<br/>Crear publisher /turtle1/cmd_vel<br/>Crear cliente /reset]
    B --> C[Esperar tecla en bucle]

    C -->|Flechas↑↓←→| D[Control manual\nsend_cmd(linear, angular)]
    C -->|1| E[SABP\n draw_SABP()]
    C -->|2| F[SFRM\n draw_SFRM()]
    C -->|3| G[Secuencia completa\n draw_FULL()]
    C -->|s,a,b,p,f,r,m| H[Dibujo de letra individual\n funciones *_continuous]
    C -->|c| I[Llamar reset_simulation()\nServicio /reset]
    C -->|h| J[Ir al origen\nmove_to_start_L(5.54,5.54)]
    C -->|q| K[Salir\nstop() y shutdown()]

    D --> C
    E --> C
    F --> C
    G --> C
    H --> C
    I --> C
    J --> C
```

---

### Organización del código fuente  
**Requerimiento del laboratorio**  
Todos los scripts y archivos de código deben estar organizados y bien comentados para facilitar su comprensión.

**Implementación en este repositorio**  
Estructura relevante:
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
- `README.md` (este archivo)  
  - Documentación del laboratorio, diagrama Mermaid y descripción del código.

El código contiene comentarios en español que explican las partes críticas (giros, tiempos, transiciones entre letras, etc.), cumpliendo con la recomendación de tener funciones bien comentadas.

---

### Conclusiones y aprendizajes  
**Requerimiento del laboratorio**  
Incluir una sección de conclusiones donde se expongan los resultados, reflexiones del equipo y aprendizajes obtenidos.

**Implementación en este repositorio**  
- Se logró implementar un nodo ROS 2 en Python que controla completamente `turtlesim` desde el teclado, respetando la restricción de no usar `turtle_teleop_key`.  
- Se diseñó un sistema de dibujo de letras continuas que mantiene tamaños y espaciados coherentes, permitiendo escribir palabras completas (**SABP** y **SFRM**) y letras individuales.  
- El uso de `turtlesim/msg/Pose` permitió incorporar control en lazo cerrado (por ejemplo, para ir a posiciones específicas y evitar cruces no deseados), mejorando la precisión de los trazos.  
- La integración del servicio `/reset` y de comandos de “home” facilita la experimentación rápida, algo muy útil en un entorno de laboratorio.  
- En conjunto, el laboratorio refuerza el entendimiento de nodos, tópicos, servicios y la interacción entre ROS 2 y Python, así como el diseño incremental de soluciones robóticas basadas en simulación.
