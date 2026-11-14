# Un evento del switch del niño (ya filtrado o crudo, según el tópico).
builtin_interfaces/Time stamp
uint8 switch_id          # 0 = switch principal, 1 = segundo switch (barrido por pasos)
bool pressed             # true al presionar, false al soltar
float32 hold_duration    # segundos que estuvo presionado (solo al soltar)
