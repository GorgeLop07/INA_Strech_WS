# Estado del barrido, para que la pantalla lo dibuje.
string mode              # "figuras" o "libre"
uint8 level              # peldaño de la escalera de control (1 a 4)
string stage             # "color", "figura", "fila", "columna", "direccion", "pintando", "dibujando", ...
string prompt            # texto o frase para el niño ("Elige un color")
string[] options         # claves de las opciones ("rojo", "circulo", "fila 2", ...)
string[] labels          # texto visible de cada opción
int32 highlighted        # opción iluminada ahora (-1 = ninguna)
bool busy                # el robot está pintando
string selection_summary # lo elegido hasta ahora ("rojo · círculo")
string target_hint       # peldaño 4: qué pide el diseño a copiar
