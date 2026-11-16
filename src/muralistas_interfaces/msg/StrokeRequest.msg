# Lo que la interacción le pide a la planeación.
uint8 SHAPE=0            # pintar una figura en una casilla
uint8 FREE_START=1       # trazo libre: empezar a moverse en una dirección (pincel apoyado)
uint8 FREE_STOP=2        # trazo libre: detenerse sin levantar el pincel
uint8 PEN_UP=3           # levantar el pincel
uint8 MOVE_TO_CELL=4     # llevar el pincel (levantado) al centro de una casilla y apoyarlo
uint8 GHOST=5            # fantasma confirmado: pintar por `points` (pen_down) o solo desplazarse (levantado)

uint8 action
string shape             # clave de la figura (SHAPE)
string color             # el marcador es uno solo (azul); vacío = el del perfil
int32 row
int32 col
float32 direction_deg    # FREE_START: 0 = derecha, 90 = arriba
float32 length           # FREE_START: largo del tramo en m (0 = hasta free_max_length o el borde)
float32 speed            # FREE_START / GHOST: m/s de la punta (0 = la del planificador)
geometry_msgs/Point[] points   # GHOST: puntos en el marco del pizarrón (m); z se ignora
bool pen_down            # GHOST: true = pintar; false = desplazarse con el marcador levantado
bool lift_at_end         # GHOST: levantar el marcador al terminar
string source            # "nino", "adulto" o "auto" (para el registro de decisiones)
