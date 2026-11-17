# Pose estimada del mural. header.frame_id es el marco de referencia (p. ej. "odom").
# Marco del mural: origen en la esquina inferior izquierda del papel, x a la derecha,
# y hacia arriba, z saliendo de la pared hacia el robot.
std_msgs/Header header
geometry_msgs/Pose pose
float32 width
float32 height
int32 markers_used
int32 corners_used
float32 rms_error        # error del ajuste (m)
float32 plane_normal_angle_deg  # diferencia entre la normal de los ArUco y la del plano de profundidad
int32 fresh_markers      # marcadores vistos en los últimos segundos que dieron la corrección actual
builtin_interfaces/Time fresh_stamp   # cuándo se calculó esa corrección (tiempo de la imagen)
float32 correction_mm    # deriva corregida de la odometría (traslación en el piso)
float32 correction_deg   # y giro
