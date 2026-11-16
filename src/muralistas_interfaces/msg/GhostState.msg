# Estado de la interacción con joystick (fantasma), para que la pantalla lo dibuje.
# Todos los puntos van en el marco del pizarrón (m).
string interaction             # J1 | J2 | J3
string mode                    # libre | rehabilitacion
string stage                   # menu | trazando | armando | esperando | preparando | paro
string prompt                  # texto para el niño
string menu_highlight          # menú levantado/pegado: "", "levantado" o "pegado"
bool pen_down                  # estado elegido para el trazo actual (pegado = pinta)
geometry_msgs/Point cursor
geometry_msgs/Point[] ghost    # fantasma actual
geometry_msgs/Point[] template_points   # plantilla (rehabilitación)
float32 dwell_progress         # J1: 0 a 1 mientras el joystick está quieto
int32 stars                    # estrellas del último intento (-1 = sin calificación)
string score_text              # "error 1.4 cm · cobertura 92%"
string level_text              # "Etapa 1 · cuadrado · asistencia media"
