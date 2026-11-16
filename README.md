# INA_Strech_WS: Pequeños Muralistas (ROS 2 Humble)

El Stretch 3 pinta en un pizarrón lo que decide un niño con parálisis cerebral, y cada sesión deja datos para seguir su motricidad fina. Funciona en simulación (MuJoCo, con el modelo oficial de `hello-robot/stretch_mujoco`) y en el robot real.

**Cómo probarlo, paso a paso, en simulación y en el robot: [PRUEBAS.md](PRUEBAS.md).**

![Escena en MuJoCo](docs/mujoco_escena.png)

## Clonar en otra máquina

`build/`, `install/`, `log/`, `.venv/` y `datos/` no van en el repo. El modelo del Stretch viene como submódulo.

```bash
git clone --recurse-submodules https://github.com/GorgeLop07/INA_Strech_WS.git
cd INA_Strech_WS
python3 -m venv --system-site-packages .venv
.venv/bin/pip install 'mujoco==3.2.6' 'numpy<2' 'opencv-python-headless==4.10.0.84'
sudo apt install ros-humble-joy python3-scipy python3-tk
source entorno.sh && mbuild
```

## La idea en dos capas

**Modo de interacción**: cómo controla el niño. Sale de la calibración.

| Modo | Para quién | Se mueve con | Confirma con |
|---|---|---|---|
| **S** | Solo controla un switch | Barrido: las opciones se iluminan una por una | El switch |
| **J1** | Una mano que no puede soltar el joystick | Joystick | Dejarlo quieto `t_quieto` (se llena un anillo) |
| **J2** | Una mano: joystick y luego switch | Joystick | Soltar el joystick y presionar el switch |
| **J3** | Las dos manos | Joystick en una mano | El switch en la otra |

**Nivel de asistencia** (alta, media, baja): cuánta ayuda da el sistema. Filtro del cursor, velocidad, imán hacia la plantilla, `t_quieto` y velocidad del robot en S. Lo fija la calibración y baja con la regla de progresión (5 de los últimos 8 intentos con 2 estrellas o más).

**Modos de juego:**
- **Figuras** (solo S): figuras preestablecidas en una cuadrícula de 4 × 3 casillas de 15 cm.
- **Libre**: en S, flecha que gira y el robot pinta hasta que el niño presiona. Con joystick, el niño elige *levantado* o *pegado*, traza un **fantasma** en la pantalla y lo confirma; el robot se desplaza o pinta.
- **Rehabilitación**: plantillas por etapa (0 línea, 1 cuadrado, 2 círculo, 3 casa). En S el niño detiene el trazo en la esquina; con joystick traza encima de la plantilla. Cada intento se califica con estrellas.

El robot nunca sigue la mano en directo: solo pinta lo que el niño confirmó, suavizado y a velocidad segura. El marcador es uno solo, azul: no hay menú de colores.

## Cómo correrlo

```bash
source entorno.sh                     # ROS 2 + .venv + lo compilado
mbuild                                # compila; después: source entorno.sh
ros2 launch muralistas_bringup calibracion.launch.py user_id:=nino_01 mando:=teclado
ros2 launch muralistas_bringup sim_mural.launch.py                                   # S, figuras
ros2 launch muralistas_bringup sim_mural.launch.py modo:=rehabilitacion              # S, parar en la esquina
ros2 launch muralistas_bringup sim_mural.launch.py interaccion:=J2 modo:=libre       # fantasma con teclado
ros2 launch muralistas_bringup sim_mural.launch.py interaccion:=J1 modo:=rehabilitacion asistencia:=alta
ros2 launch muralistas_bringup sim_mural.launch.py perfil:=datos/perfil_nino_01_<fecha>.yaml
```

**Argumentos:** `perfil`, `interaccion` (S, J1, J2, J3), `modo` (figuras, libre, rehabilitacion), `nivel` (peldaño 1 a 4 de figuras), `asistencia`, `etapa`, `mando` (`teclado` en simulación, `pro_controller` en el robot), `ui`, `viewer`, `rviz`, `board_frame`, `auto_startup`. Para el robot real se usa `real_mural.launch.py parte:=robot|laptop|todo` (ver [PRUEBAS.md](PRUEBAS.md)).

**Al arrancar (~40 s)** el robot:
1. se pone en su pose inicial;
2. mira a la izquierda, al centro y a la derecha para ubicar los 4 ArUco del pizarrón;
3. calibra la punta del marcador con la nube 3D;
4. queda "listo". La interacción no empieza hasta entonces.

**Teclas:**
- **Pantalla del barrido (S):** espacio = switch, a = elige el adulto, n = segundo switch, p o Esc = PARO, r = reanudar.
- **Pantalla del fantasma (J), con `mando:=teclado`:** flechas = joystick, espacio = switch (ZR), Enter = A (confirmar), Retroceso = B (borrar), p o Esc = PARO, r = reanudar.

**Mando:** Nintendo Switch Pro Controller por USB, mapeado en `config/mandos/pro_controller.yaml`: joystick izquierdo = cursor, A = confirmar, B = borrar, ZR = switch, + = paro de respaldo. `joystick.launch.py` prueba solo el mando.

**Seguridad:** el marcador se levanta solo si pasa 10 s apoyado sin moverse (`pen_idle_lift`). El robot dibuja a 5 cm/s y se desplaza a 10 cm/s, muy por debajo de los 250 mm/s de velocidad reducida de ISO 10218. El paro principal es el botón del robot.

## Registro de cada sesión (`datos/`)

| Archivo | Qué guarda |
|---|---|
| `decisiones_<usuario>_<fecha>.csv` | Cada decisión, su fuente (niño, adulto, automática) y su tiempo de respuesta: % de decisiones propias |
| `intentos_<usuario>_<fecha>.csv` | Cada intento de rehabilitación: estrellas, error, cobertura, duración, LDLJ, SPARC, MSJ, cambio de progresión |
| `joystick_<usuario>_<fecha>.csv` | Joystick crudo y filtrado, cursor y botones a 100 Hz |
| `trazos_<usuario>_<fecha>.jsonl` | Cada fantasma confirmado: el crudo, el mostrado (con imán) y su calificación |
| `punta_<usuario>_<fecha>.csv` | La punta del marcador en el pizarrón y si está apoyada (10 Hz) |
| `progreso_<usuario>_<fecha>.yaml` | Etapa y asistencia sugeridas para la siguiente sesión |
| `calibracion_<usuario>_<fecha>.json`, `perfil_<usuario>_<fecha>.yaml` | Todo lo medido en C1 a C5 y el perfil sugerido |

## Paquetes

| Paquete | Qué hace |
|---|---|
| `muralistas_interfaces` | Mensajes, servicios y acciones (`StrokeRequest`, `ScanState`, `GhostState`, `Attempt`, `Board`, `Decision`, `DrawStroke`, `CalibrateTool`, `SetBrushColor`) |
| `muralistas_common` | Sin ROS: pizarrón y zona de trabajo (`layout`), figuras, plantillas, métricas (LDLJ, SPARC, MSJ, calificación, progresión), niveles de asistencia |
| `muralistas_sim` | MuJoCo: Stretch 3, pared, pizarrón con 4 ArUco, pincel con resorte y tinta. Expone las mismas interfaces que el driver real |
| `muralistas_perception` | Nube 3D de la cabeza; pose del pizarrón (esquinas de ArUco en 3D + plano); segmentación del marcador por color (verde en sim, azul en el robot) y calibración de la punta; `generar_arucos` para imprimirlos |
| `muralistas_planning` | Figura + casilla, tramo libre (con largo y velocidad) o fantasma (simplificar, redondear, remuestrear) → trazos en el pizarrón |
| `muralistas_control` | Trazos → base + elevador + brazo; acercar, apoyar, pintar, levantar, desplazarse sin pintar; levantar solo tras 10 s quieto; paro; rutina de arranque. `prueba_articulaciones` para el robot real |
| `muralistas_hri` | Filtro del switch, menú de barrido (S: figuras, libre, rehabilitación), entrada del joystick, fantasma (J1, J2, J3), pantallas del niño, calibración C1 a C7 y registro |
| `muralistas_bringup` | Launch (`sim_mural`, `real_mural`, `calibracion`, `joystick`), `config/` (pizarrón, simulación, control, percepción, robot real), `config/perfiles/` (uno por niño) y `config/mandos/` |

**Flujo:** switch o joystick → `hri` → `planning` → `control` → simulador o driver del robot. La percepción da la pose del pizarrón y la punta calibrada.

## Resultados en simulación (13-nov-2026, papel de 3 m con 14 ArUco)

Se midieron antes del cambio al pizarrón de 1.5 × 1.0 m con 4 ArUco. Hay que repetirlos con el pizarrón nuevo (ver [PRUEBAS.md](PRUEBAS.md)).

| Qué | Resultado |
|---|---|
| Pose del mural contra la verdad | 1.7 a 3.4 mm y ≤ 0.05° |
| Punta del pincel calibrada | 2.3 mm de error (sin calibrar: 14 mm) |
| Error del trazo contra lo planeado | Medio 1.2 a 2.1 mm, máximo 4 mm |
| Velocidad de la simulación | ~1.0× tiempo real con todo el sistema |

**Robot real (14-nov-2026):** las 7 articulaciones, la base (5 cm) y la trayectoria combinada en modo trajectory responden por ROS con errores menores a 1 mm en elevador y brazo.

## Pruebas unitarias

```bash
PYTHONPATH=src/muralistas_common:src/muralistas_hri:src/muralistas_control:src/muralistas_planning:$PYTHONPATH python -m pytest -q src/*/test
```

48 pruebas: filtros del joystick, métricas y plantillas, menú de barrido (incluida la rehabilitación con switch), fantasma (J1, J2, rehabilitación), calibración, planeación y mapeo cartesiano.

## Decisiones de la simulación (para no tropezar de nuevo)

- **El lidar está apagado** (`physics.lidar: false`): costaba 28 veces más que el resto de la física.
- **Las imágenes se arman con `array.array('B', ...)`.** Asignar `bytes` a `Image.data` en Humble valida byte por byte (~50 ms por imagen).
- **El papel es solo visual** y la punta toca la pared; con el papel como caja de colisión la punta quedaba atrapada (~60 N en vez de ~2 N).
- **El elevador y el brazo simulados llevan un lazo** de prealimentación de velocidad más integral, que imita a los motores reales.
- **Los argumentos del launch van en un YAML con el nombre del nodo** (y los parámetros con punto, anidados). Un diccionario se pasa como `/**` y pierde contra el perfil.
- **Ningún campo de mensaje se llama `template`**: es palabra reservada de C++ y rompe la compilación de las interfaces.
