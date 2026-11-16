# Un intento calificado (rehabilitación), para el registro.
builtin_interfaces/Time stamp
string user_id
string interaction       # S | J1 | J2 | J3
int32 stage              # etapa (0 línea, 1 cuadrado, 2 círculo, 3 casa)
string template_name     # nombre de la plantilla
string assistance        # alta | media | baja
int32 stars
float32 mean_error       # m: joystick = error medio al trazo; S = error medio de paro (absoluto)
float32 coverage         # 0 a 1 (S: tramos completados / tramos)
float32 duration         # s
float32 ldlj             # suavidad de la señal cruda (NaN en S)
float32 sparc
float32 msj
string detail            # JSON con lo demás (errores por tramo, direcciones equivocadas, ...)
string progression       # cambio que provocó: baja_asistencia, sube_etapa, sube_asistencia, completo o ""
