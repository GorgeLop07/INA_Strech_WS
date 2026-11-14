# Lo que la interacción le pide a la planeación.
uint8 SHAPE=0            # pintar una figura en una casilla
uint8 FREE_START=1       # trazo libre: empezar a moverse en una dirección (pincel apoyado)
uint8 FREE_STOP=2        # trazo libre: detenerse sin levantar el pincel
uint8 PEN_UP=3           # levantar el pincel
uint8 MOVE_TO_CELL=4     # llevar el pincel (levantado) al centro de una casilla y apoyarlo

uint8 action
string shape             # clave de la figura (SHAPE)
string color             # clave del color
int32 row
int32 col
float32 direction_deg    # FREE_START: 0 = derecha, 90 = arriba
string source            # "nino", "adulto" o "auto" (para el registro de decisiones)
