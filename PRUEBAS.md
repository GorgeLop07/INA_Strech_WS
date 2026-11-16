# Cómo probarlo: simulación, robot real y siguientes pasos

Fecha: 15-nov-2026. Esta guía cubre todo lo que se agregó siguiendo la planeación:
- pizarrón de 2.40 × 1.18 m (parametrizable) con 6 ArUco de 15 cm y zona de trabajo de 80 × 60 cm;
- barrido de cabeza calculado con las medidas del pizarrón, con el brazo abajo para que no tape;
- marcador azul, sin colores;
- modos de interacción S, J1, J2 y J3 y niveles de asistencia;
- fantasma con ciclo levantado / pegado;
- rehabilitación con plantillas, imán, estrellas y progresión;
- calibración C1 a C7;
- registro de la sesión;
- launch del robot real;
- levantado automático del marcador.

**Estado de cada parte:**

| Parte | Estado |
|---|---|
| Pruebas unitarias | ✅ Pasan las 49 |
| Compilación | ✅ Compilan los 8 paquetes |
| Launch | ✅ Arman los nodos correctos en cada combinación (revisado sin lanzarlos) |
| Simulación, S1 (arranque con el pizarrón nuevo) | ✅ Pasa con 240 × 118 (15-nov, corrida de Jorge): 6 de 6 marcadores al primer barrido |
| Simulación, S2 a S5 | ✅ Pasaron (15-nov) con el pizarrón de 1.5 × 1.0; conviene repetir S2 y S3 con la zona nueva (casillas de 20 cm, plantillas de 40 cm) |
| Simulación, S6 (J3 con el Pro Controller) y S7 (calibración) | ✅ Probadas a mano por Jorge (15-nov); salió un error en el retardo del switch, ya corregido |
| Pantallas nuevas (fantasma, calibración) | ✅ Abiertas en S4–S7 (capturas en `docs/`) |
| Robot real con estos cambios | ❌ No lo he corrido |

Por eso conviene seguir el orden de esta guía: primero la simulación y después el robot, paso por paso.

---

## 0. Antes de todo

```bash
cd ~/Documents/Heal_Then_Proyectos/ICORR/INA_Strech_WS
sudo apt install ros-humble-joy python3-scipy python3-tk     # si falta alguno
source entorno.sh && mbuild && source entorno.sh
PYTHONPATH=src/muralistas_common:src/muralistas_hri:src/muralistas_control:src/muralistas_planning:$PYTHONPATH python -m pytest -q src/*/test
```

**Qué esperar:** `49 passed`. Sale un aviso de SciPy sobre la versión de NumPy; no afecta.

---

## 1. Simulación (no necesita robot ni control)

Todos se lanzan con `ros2 launch muralistas_bringup sim_mural.launch.py ...`. En simulación el mando por defecto es el **teclado**. Cada prueba cierra con Ctrl+C **una sola vez**.

### S1. Arranque con el pizarrón nuevo

```bash
ros2 launch muralistas_bringup sim_mural.launch.py
```

**Qué esperar:**
- En MuJoCo, un pizarrón de 2.40 × 1.18 m con 6 ArUco de 15 cm (las 4 esquinas y el centro de abajo y de arriba), centrado frente al robot. Su borde de abajo queda a 0.20 m (`bottom_height`), así que la zona de trabajo (80 × 60 cm) cae entre 0.45 y 1.05 m de alto.
- En el log del control: `Barrido de cabeza (pan, tilt): (-0.60, -0.32), (-1.57, -0.93), (-1.57, -0.03), (-2.54, -0.32)`. Son 4 poses: esquinas de la izquierda, centro abajo, centro arriba y esquinas de la derecha.
- En unos 40 s el robot hace su pose inicial, **baja el elevador a 0.2 m** y recoge el brazo (para no tapar los ArUco de abajo), barre la cabeza por esas 4 poses y regresa el brazo a la pose de trabajo. Calibra el pincel y el estado queda en **"listo"**.
- La pantalla del barrido empieza en **"Elige una figura"**, sin pregunta de color.

**Para revisar en otra terminal:**

```bash
ros2 topic echo /perception/board --once
```

Debe salir `markers_used: 6` y `rms_error` de pocos milímetros. En el log: `Mural ubicado (6 de 6 marcadores)`.

**Resultado con 240 × 118 y 6 ArUco (15-nov, corrida de Jorge):**
- Barrido calculado: `(-0.60, -0.32), (-1.57, -0.93), (-1.57, -0.03), (-2.54, -0.32)`, con el elevador abajo.
- `Mural ubicado (6 de 6 marcadores)` al primer barrido, sin repetirlo. Arranque completo en 25 s.
- Punta calibrada con dispersión de 0.2 mm.

**Resultado con 1.5 × 1.0 y 6 ArUco (15-nov, corrida de Jorge, antes de bajar el brazo en el barrido):**
- Vio 5 de 6, repitió el barrido y arrancó con `Mural ubicado (5 de 6 marcadores)`.
- En la imagen de la cámara se vio la causa: con el elevador a 0.6 m (pose de trabajo), el brazo y el pincel quedan justo entre la cámara y el ArUco de abajo al centro. Como ese marcador está en la misma línea que el brazo, ninguna pose de cabeza lo destapa.
- Arreglo: durante el barrido el elevador baja a 0.2 m y el brazo se recoge (`scan_pose` en `config/control.yaml`). Así el brazo queda debajo de la línea de vista. Con eso salió 6 de 6 (resultado de arriba).

**Resultado con 4 ArUco (15-nov, antes del cambio a 6):**
- Arranque en 25 s; log `Mural ubicado (4 de 4 marcadores)`.
- Pizarrón a **3.5 mm y 0.03°** de la verdad de la simulación; error de ajuste 1.3 mm.
- Percepción a 4 cuadros/s; la pantalla pregunta "Elige una figura".

Para llegar ahí hubo que corregir dos cosas:
1. La verificación del plano (`check_plane`) usaba toda la pared y tardaba 3.5 s por cuadro, así que el barrido solo veía 2 marcadores. Ahora usa solo el pizarrón (máximo 3000 puntos, 0.18 s) y corre cada 2 s.
2. Reiniciar el pizarrón mientras se procesaba un cuadro tumbaba la percepción. Ahora hay un candado.

Además, el control espera a la percepción antes de barrir, se queda 1 s por pose y repite el barrido si no ve todos los marcadores del pizarrón (6 con `markers: seis`).

**Si ve menos de 6:**
- Mira la ventana "Cámara (ArUco y pincel)" de RViz mientras barre: se ve qué marcador no entra o qué lo tapa.
- Si no entra en la imagen, sube `scan.margin_deg` (más poses, más centradas) o corrige `scan.wall_distance`.
- Si lo tapa el brazo, baja `scan_pose`.
- Para seguir probando lo demás mientras tanto, lanza con `board_frame:=board_gt auto_startup:=false`.

### S2. Modo S, figuras y libre

```bash
ros2 launch muralistas_bringup sim_mural.launch.py nivel:=3
ros2 launch muralistas_bringup sim_mural.launch.py nivel:=2
ros2 launch muralistas_bringup sim_mural.launch.py modo:=libre
```

- **Peldaño 3:** figura, luego fila, luego columna. La figura se pinta en una casilla de 20 cm dentro de la zona de 80 × 60 cm.
- **Peldaño 2:** eliges la figura y la casilla se elige sola.
- **Libre:** fila y columna. Luego la flecha gira; espacio arranca el trazo y espacio lo detiene. "Levantar" termina.
    - Si dejas pasar más de 10 s eligiendo dirección, el marcador se levanta solo (en el log: `Pincel apoyado 10 s sin moverse`). El siguiente tramo lo vuelve a apoyar en el mismo lugar.

**Resultado (15-nov, con un "niño automático" que presiona el switch en `/hri/switch_raw` cuando se ilumina la opción):**

| Prueba | Resultado |
|---|---|
| Peldaño 3: círculo, fila 2, casilla 2 | 151/151 puntos de tinta dentro de la casilla; error medio 1.6 mm (máx. 4.6) contra lo planeado, incluido el error del pizarrón |
| Peldaño 2: sol y zigzag | Casillas automáticas (0,0) y (0,1), registradas como "auto"; sol 9/9 trazos, error 1.7 mm; zigzag error 1.4 mm |
| Libre: pincel a la casilla | A menos de 1 mm del centro |
| Libre: → 2 s y ↑ 1.5 s | 9.7 cm a −1° y 8.1 cm a 90° |
| Libre: 12 s sin elegir | Se levantó solo a los 10.6 s |
| Libre: ← después de levantarse | Volvió a apoyar a 0.2 cm de donde quedó |
| Levantar | Pincel arriba, regresa a "fila" |

Ojo: después de levantarse solo, el siguiente tramo tarda ~1.3 s en volver a apoyar antes de pintar.

### S3. Modo S, rehabilitación (detener en el momento justo)

```bash
ros2 launch muralistas_bringup sim_mural.launch.py modo:=rehabilitacion etapa:=1
```

**Qué esperar:**
1. "¡Presiona para empezar!" → espacio. El robot lleva el marcador al inicio del cuadrado (40 cm) y lo apoya.
2. La flecha gira. Si eliges una dirección equivocada, se registra y vuelve a girar. La correcta para el primer tramo es **→ derecha**.
3. El robot avanza a 3 cm/s (asistencia media): cada lado de 40 cm tarda unos 13 s. Presiona espacio cuando llegue a la esquina. El tramo se alarga 5 cm para que te puedas pasar.
4. Después de los 4 tramos levanta el marcador. En la línea de abajo se ve "Último intento: ★★ · error 1.4 cm" y arriba "Etapa 1 · Cuadrado · asistencia media · Tramo x de 4".
5. En `datos/intentos_nino_01_<fecha>.csv` aparece una fila por intento, con los errores de paro de cada tramo en la columna `detalle`.

**Progresión:** a los 8 intentos, si 5 tienen 2 estrellas o más, baja la asistencia (media → baja: el robot va a 5 cm/s). Ya en baja, sube a la etapa 2 (octágono) con asistencia alta.

**Resultado (15-nov, niño automático que para a propósito antes o después de la esquina):**

| Prueba | Resultado |
|---|---|
| Inicio | El pincel se apoya a 1 mm del inicio de la plantilla |
| Dirección equivocada (↑ en el primer tramo) | Se registra y vuelve a preguntar |
| Velocidad del robot (asistencia media) | 2.98 a 3.00 cm/s medida, contra 3 cm/s configurada |
| Paros buscados +1, −1, 0, +2 cm | Medidos +1.0, −1.1, −0.2, +1.8 cm → error medio 1.0 cm, ★★ |
| Paros buscados en la esquina | −0.2, −0.3, −0.2, −0.1 cm → ★★★ |
| Tramo sin parar | El robot llega al final del tramo alargado: +4.6 cm, cuenta como "no paró" y tramos completados 0.75 |
| Registro | `intentos_*.csv` con una fila por intento; errores por tramo, direcciones equivocadas, tiempos de reacción y de elección |

Se corrigió que el "tiempo de reacción" medía la espera del barrido. Ahora:
- **reacción** = desde que se ilumina la opción elegida hasta la presión;
- **elección** = desde la pregunta.

**Pendiente de decidir:** un intento con un tramo sin parar sacó ★★, porque el promedio de los 4 errores fue 1.2 cm. ¿Debería un "no paró" limitar el intento a ★?

### S4. Joystick J2, modo libre (con teclado)

```bash
ros2 launch muralistas_bringup sim_mural.launch.py interaccion:=J2 modo:=libre
```

Se abre **"Pequeños muralistas · joystick"** en lugar de la pantalla del barrido. Haz clic en ella para que reciba el teclado.

1. **Menú:** flecha → ilumina "Pegado" y flecha ← ilumina "Levantado". Espacio (el switch) elige.
2. **Trazo:** con las flechas el cursor amarillo traza el fantasma desde la punta roja.
    - Pegado se dibuja azul; levantado, punteado.
    - El cursor va como máximo a 18 cm/s y no sale de la zona.
3. **Confirmar:** espacio corto. El robot pinta (pegado) o solo se desplaza (levantado). En la pantalla, lo pintado se dibuja en azul siguiendo la punta real.
4. **Borrar:** Retroceso (B), o mantener espacio más de 1 s y soltar.
5. **Fantasma corto:** uno de menos de 3 cm no se confirma.

Después de pintar regresa al menú. Si eliges pegado otra vez y empiezas donde quedó la punta, el marcador no se levanta entre trazos.

**Resultado (15-nov, con un mando automático que publica `/joy` con el mapeo del Pro Controller):**

| Prueba | Resultado |
|---|---|
| Joystick → en el menú | Ilumina "pegado"; el switch lo elige y el cursor aparece en la punta |
| Fantasma de 1.4 cm | No se confirma (mínimo 3 cm) |
| Fantasma de 25 cm (→ y ↑) + switch corto | El robot lo pinta: tinta a 1.5 mm del fantasma (máx. 3.8 mm); 79 puntos simplificados a 27 |
| Levantado (joystick ←) | Se desplaza sin tinta y queda a 3 mm del final del fantasma |
| Presión larga (1.3 s) y botón B | Las dos borran el fantasma |
| Pantalla | Menú, fantasma, "El robot está pintando" y la tinta siguiendo la punta real (captura en `docs/pantalla_fantasma.png`) |

Se corrigieron tres cosas:
1. **El teclado como mando fallaba de inmediato.** Mandaba el eje como entero cuando no había teclas, ROS lo rechazaba y el ciclo que publica `/joy` se detenía.
2. **El menú levantado / pegado tapaba la parte de abajo del pizarrón.** Ahora va en su propia franja.
3. **En el menú, el cursor se movía con el joystick.** Ahora se queda en la punta.

No hay herramienta para mandar teclas a la ventana desde aquí: hay que probar las flechas a mano.

### S5. Joystick J1 en rehabilitación (confirmar quieto)

```bash
ros2 launch muralistas_bringup sim_mural.launch.py interaccion:=J1 modo:=rehabilitacion asistencia:=alta etapa:=0
```

1. El robot se desplaza al inicio de la línea. Aparece la plantilla gris punteada y el cursor en su inicio.
2. Recorre la línea con las flechas.
    - Con asistencia alta el imán es 0.5: el fantasma se ve jalado hacia la plantilla.
    - La calificación usa el trazo **sin** imán.
3. Suelta las flechas: un anillo verde se llena en **2 s** (t_quieto de asistencia alta). El cursor se pone verde y el robot espera 1 s; si te mueves en ese segundo, se cancela.
4. El robot pinta el fantasma y levanta. Arriba salen las estrellas y "error x cm · cobertura y%". Luego regresa al inicio para el siguiente intento.
5. Revisa `datos/trazos_nino_01_<fecha>.jsonl`: una línea por intento con el trazo crudo, el mostrado y LDLJ, SPARC y MSJ.

**Nota:** con el teclado las flechas son todo o nada, así que la suavidad que sale no dice nada del niño. Sirve para probar el flujo, no para medir.

**Resultado (15-nov, mando automático):**

| Prueba | Resultado |
|---|---|
| Inicio | El robot se desplaza al inicio de la línea; el cursor aparece ahí |
| Anillo de J1 (asistencia alta) | Se llena en 2.0–2.1 s |
| Retraso antes de arrancar | 1.03 s (medido con un proceso que solo escucha las etapas) |
| Moverse durante ese segundo | Cancela y regresa a trazar |
| Imán 0.5 | Un trazo desviado 2.3 cm se ve desviado 1.15 cm, pero la calificación usa el crudo: error 12.3 mm, cobertura 66%, 0 estrellas |
| Trazos cerca de la plantilla | Línea vertical: 2.1 mm, 100% → ★★★. Cuadrado (etapa 1): 3–4 mm, 100% → ★★★ |
| Tinta contra el fantasma | 1.9 mm |
| Plantilla siguiente | Alterna línea → línea vertical, y vuelve a empezar en su inicio |
| Registro | `intentos_*.csv` y `trazos_*.jsonl`, una fila por intento |
| Pantalla | Plantilla gris, fantasma azul, anillo verde, estrellas y puntaje (captura en `docs/pantalla_rehabilitacion.png`) |

Se corrigió que la plantilla se dibujaba encima de la tinta y la tapaba.

### S6. J3 con el Pro Controller en simulación

```bash
ros2 launch muralistas_bringup sim_mural.launch.py interaccion:=J3 modo:=libre mando:=pro_controller
```

Es igual que S4, pero con el control: el joystick izquierdo mueve, ZR es el switch, A confirma, B borra y + hace paro. Con `r` en la pantalla se reanuda.

### S7. Calibración (sin robot ni simulación)

```bash
ros2 launch muralistas_bringup calibracion.launch.py user_id:=prueba mando:=teclado
ros2 launch muralistas_bringup calibracion.launch.py user_id:=prueba macs:=3      # con el Pro Controller
```

Enter empieza cada prueba y `s` la salta:
- **C1:** presiona el switch (espacio o ZR) cuando se prenda la luz verde; son 10 luces.
- **C2:** 10 s sin mover el joystick.
- **C3:** 16 estrellas; empuja hacia cada una y suelta.
- **C4:** sigue el punto amarillo por 3 vueltas.
- **C5:** joystick a un lado y switch con la otra mano; 3 intentos.

**Qué esperar al final:** "Resultado: modo J2, asistencia media" (o el que salga), la razón, lo que sugiere MACS y las rutas a `datos/calibracion_prueba_<fecha>.json` y `datos/perfil_prueba_<fecha>.yaml`.

**Resultado (15-nov, Jorge con el Pro Controller):**
- C1: 10 de 10, reacción mediana 0.32 s.
- C2: temblor 0.0 (el control en reposo da valores exactos).
- C3: alcance de 0.80 a 0.89 en las 8 direcciones.
- C4: 24% de error.
- C5: 3 de 3, así que el modo sale J3, asistencia media. Con MACS 3 avisa que no coincide.
- El perfil generado funcionó con `sim_mural.launch.py perfil:=...`.

**Error encontrado y corregido:** el retardo del switch salió de 7.8 s. Se contaba como "rebote" la presión de la luz siguiente. Ahora un rebote es una segunda presión a menos de 0.5 s, y el retardo queda entre 0.3 y 1.0 s. Los perfiles generados antes de este arreglo traen `post_acceptance_delay` mal: hay que cambiarlo a 0.3 o calibrar de nuevo.

**Usar ese perfil:**

```bash
ros2 launch muralistas_bringup sim_mural.launch.py perfil:=datos/perfil_prueba_<fecha>.yaml
```

Ya trae la interacción, la asistencia, la zona muerta, las 8 ganancias, el filtro One Euro y el periodo del barrido.

---

## 2. Robot real (paso por paso, con tu OK en cada uno)

Como en las pruebas del 14-nov: en cada paso digo el comando, qué se mueve y qué esperar, y se corre solo cuando des el OK.

### F0. Preparar (no se mueve nada)

**En el robot:**

```bash
cd ~/INA_Strech_WS && git pull
source /opt/ros/humble/setup.bash && source ~/ament_ws/install/setup.bash
colcon build --symlink-install --packages-ignore muralistas_sim && source install/setup.bash
python3 -c "import cv2; print(cv2.__version__, hasattr(cv2.aruco, 'ArucoDetector'))"
```

- **Sin simulación:** en el robot no se compila `muralistas_sim` (necesita MuJoCo). `--packages-ignore` hace como si no existiera; `muralistas_bringup` ya no depende de él.
- **OpenCV:** la última línea debe decir 4.7 o más y `True`. Si dice `False`, la percepción no puede correr en el robot y se corre en la laptop con `parte:=laptop`; lo vemos en ese momento.
- **Red:** en la laptop y en el robot, el mismo `export ROS_DOMAIN_ID=<n>`. Con el driver corriendo, `ros2 topic list` en la laptop debe mostrar `/joint_states` del robot.

### F1. El pizarrón físico (no se mueve nada)

1. **Imprimir los marcadores:**

    ```bash
    ros2 run muralistas_perception generar_arucos --salida ~/arucos
    ```

    Salen 6 hojas con cuadros de 15 cm (`marker_size`). Imprime al 100% y mide que el cuadro negro mida 15.0 cm.
2. **Pegarlos** con su centro a 10 cm (`margin/2`) del borde de abajo o de arriba, en sentido antihorario: ID 0 abajo a la izquierda, 1 abajo al centro, 2 abajo a la derecha, 3 arriba a la derecha, 4 arriba al centro, 5 arriba a la izquierda. Los de las esquinas van también a 10 cm del borde lateral; los del centro, a la mitad del ancho (1.20 m). Cada hoja dice dónde va.
3. **Medir la altura del pizarrón.** Mide la altura del borde de abajo sobre el piso y ponla en `bottom_height` en `config/mural.yaml`. Con eso la zona de trabajo se coloca sola entre 0.45 y 1.05 m del piso. Si el pizarrón no mide 2.40 × 1.18 m, sigue la sección 3.
4. **Colocar el robot** paralelo al pizarrón, centrado a lo ancho, con el centro de la base a 0.75 m de la pared (`scan.wall_distance` en `config/control.yaml`; si lo pones a otra distancia, cámbialo ahí).

### F2. Driver y cámara (no se mueve nada)

En una terminal del robot, el driver:

```bash
ros2 launch stretch_core stretch_driver.launch.py mode:=trajectory broadcast_odom_tf:=True
```

En otra terminal del robot, la cámara:

```bash
ros2 launch stretch_core d435i_high_resolution.launch.py
```

Si su versión de `stretch_core` tiene otro nombre para el launch de la cámara de la cabeza, usamos ese.

Comprobaciones:

```bash
ros2 topic hz /camera/color/image_raw
```

```bash
ros2 topic hz /camera/aligned_depth_to_color/image_raw
```

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

**Qué esperar:** las imágenes a 10–30 Hz, y `odom → base_link` con números. Ese marco es el que faltaba el 14-nov.

### F3. Percepción, planeación y control en el robot (sí se mueve)

```bash
ros2 launch muralistas_bringup real_mural.launch.py parte:=robot
```

**Qué se mueve:**
- El robot va a la pose inicial (elevador 0.6 m, brazo casi recogido, muñeca derecha).
- Baja el elevador a 0.2 m, barre la cabeza (4 poses con el pizarrón de 240 × 118) y regresa el brazo.
- Si ve el marcador, calibra la punta.
- La base no se mueve.

**Qué esperar:**
- En el log: "listo" (o "sin mural" si no encontró el pizarrón).
- `ros2 topic echo /perception/board --once` debe dar `markers_used: 6` (con 4 o 5 también se ubica, pero hay que ajustar `head_scan`).
- `ros2 topic echo /perception/tool_offset --once` debe dar la punta del plumón en `link_grasp_center`.

Compara la punta con la regla (punta del marcador respecto a la pinza) y escribe el valor medido en `config/control_real.yaml`, en `tool_offset`.

**Si no segmenta el plumón:** hay que ajustar `brush.hsv_low` y `brush.hsv_high` en `config/perception_real.yaml` a la luz del salón, y medir `tip_beyond_body` y `body_radius`.

### F4. Primer contacto con el pizarrón (sí se mueve, y toca)

En la laptop, con F3 corriendo en el robot:

```bash
ros2 launch muralistas_bringup real_mural.launch.py parte:=laptop interaccion:=S modo:=figuras nivel:=3
```

Elige **Línea**, fila "En medio", casilla 2.

**Qué se mueve:** base, elevador y brazo. El robot se acerca a 3 cm del pizarrón, apoya el marcador 3 mm (`press_depth: 0.003`), pinta una línea de unos 12 cm y lo levanta.

**Qué esperar:** una línea azul dentro de la casilla.
- Si el marcador no pinta, sube `press_depth` de 1 en 1 mm.
- Si empuja de más, bájalo.
- Ten a la mano el paro del robot.

### F5. Joystick en el robot (sí se mueve)

```bash
ros2 launch muralistas_bringup real_mural.launch.py parte:=laptop interaccion:=J2 modo:=libre
```

Es el mismo flujo que S4, con el Pro Controller. Prueba un trazo levantado y otro pegado. Luego deja el marcador apoyado 10 s: debe levantarse solo.

### F6. Rehabilitación en el robot (sí se mueve)

```bash
ros2 launch muralistas_bringup real_mural.launch.py parte:=laptop interaccion:=S modo:=rehabilitacion etapa:=0
```

```bash
ros2 launch muralistas_bringup real_mural.launch.py parte:=laptop interaccion:=J1 modo:=rehabilitacion asistencia:=alta
```

Es el mismo flujo que S3 y S5. El pizarrón no se borra solo: entre intentos lo borra el adulto, o se dibuja encima.

---

## 3. Si el pizarrón cambia de tamaño

El pizarrón de pruebas es de **2.40 × 1.18 m** y así viene configurado. Para otro tamaño casi todo se ajusta solo: la percepción, la simulación, el planeador, las pantallas, el control y **el barrido de la cabeza** salen de `config/mural.yaml`. Los ArUco del centro se recolocan solos a la mitad del ancho, y la simulación centra el pizarrón y lo pone a `bottom_height`.

### Lo que hay que cambiar

En `config/mural.yaml`:

```yaml
      width: 2.40           # ancho (m)
      height: 1.18          # alto (m)
      bottom_height: 0.20   # altura del borde de abajo sobre el piso (m), medida en el salón
```

Y en `config/control.yaml`, solo si el robot no queda a 0.75 m de la pared:

```yaml
    scan:
      wall_distance: 0.75   # del centro de la base a la pared (m)
```

Después: `mbuild && source entorno.sh` y S1 (o F3 en el robot). En el log salen las poses calculadas (`Barrido de cabeza (pan, tilt): ...`) y debe salir `Mural ubicado (6 de 6 marcadores)`.

### Lo que no cambia solo (y por qué)

- **La zona de trabajo se queda en 80 × 60 cm**, centrada a lo ancho y con su centro a 0.75 m del piso (`work_center_z`). No crece con el pizarrón porque depende del niño y del robot:
    - el elevador lleva la punta de ~0.12 a ~1.20 m del piso, así que la zona termina en 1.05 m para dejar margen;
    - horizontalmente la mueve la base, y entre más se mueve, más deriva acumula la odometría hasta volver a ver un ArUco;
    - la cuadrícula del barrido (4 × 3) da casillas de 20 cm; más columnas alargan el barrido del switch.
- **Para una zona más grande** (por ejemplo 1.20 × 0.60 m), cambia `work_width`/`work_height`. Para mantener casillas cuadradas, ajusta también `cols`/`rows` (casilla = ancho / columnas; 1.20 m con 6 columnas da 20 cm). `template_size` es el lado de las plantillas de rehabilitación.
- **Si el pizarrón está muy alto** (borde de abajo arriba de ~0.45 m), la zona ya no cabe a su altura: se recorta sola para no pisar el margen de los marcadores y queda más arriba de lo planeado. El robot no alcanza a pintar arriba de ~1.20 m.
- **Tamaño de los marcadores:** con 2.40 m de ancho, las esquinas quedan a ~1.35 m de la cámara y a 56° de lado, por eso son de 15 cm. Para un pizarrón chico (1.5 m o menos) bastan 10 cm: `marker_size: 0.10` y `margin: 0.14` (el margen es el marcador más 2.5 cm de blanco de cada lado).

### Cómo se calcula el barrido

`muralistas_common/head_scan.py` calcula, para cada ArUco, a dónde tiene que mirar la cabeza: el robot está centrado a `wall_distance` de la pared y la cámara está a 1.30 m del piso. La cámara va girada: ve ±21° a los lados y ±34° arriba y abajo. Luego junta los marcadores en el menor número de poses que los dejen a 8° o más del borde de la imagen (`scan.margin_deg`), contando lo que mide cada marcador.

| Pizarrón | Poses (pan, tilt) |
|---|---|
| 2.40 × 1.18, borde a 0.20 m | (−0.60, −0.32), (−1.57, −0.93), (−1.57, −0.03), (−2.54, −0.32) |
| 2.00 × 1.50, borde a 0.10 m | 6 poses: izquierda, centro y derecha, cada una abajo y arriba |
| 1.50 × 1.00, borde a 0.225 m, marcadores de 10 cm | (−0.83, −0.46), (−1.57, −0.55), (−2.31, −0.46) |

Para poner las poses a mano: `head_scan_auto: false` y la lista en `head_scan`.

**Por qué el brazo baja durante el barrido:** el ArUco de abajo al centro queda en la misma línea que el brazo. Con el elevador a la altura de trabajo, el pincel lo tapa desde cualquier ángulo de la cabeza. Con el elevador a 0.2 m y el brazo recogido (`scan_pose: [0.2, 0.0]`), el brazo queda debajo de la línea de vista.

---

## 4. Lo que no está hecho o no está probado

- **J3 continuo** (opcional en la planeación): mantener el switch para pintar mientras el joystick dibuja varios trazos de una vez. **No está implementado**; J3 funciona igual que J2.
- **Borrar en J1 en modo libre** (quieto en el inicio del fantasma) choca con dibujar una figura cerrada. Para cerrar una figura en J1, hay que parar a 2 cm o más del inicio.
- **Altura del pizarrón:** el primer pizarrón real empezaba a 0.925 m del piso, y el robot solo alcanza con la punta hasta ~1.19 m (elevador a 1.08 m + 0.115 m de la pinza): no cabía la zona de trabajo. Se va a probar con otro de 240 × 118 que empieza desde el piso. `bottom_height` se queda en 0.20, como en la simulación. Si el real empieza en 0, la zona se recorre sola hacia abajo para no pisar el margen y queda entre ~0.38 y 0.98 m del piso (dentro del alcance). Regla: el borde de abajo del pizarrón tiene que estar a 0.45 m o menos.
- **El pizarrón de 240 × 118 pasó S1 en simulación** (6 de 6). Falta S2–S5 con la zona nueva y todo en el robot.
- **Los umbrales de la calibración, la asistencia y las estrellas** son propuestas de la planeación: falta validarlos con terapeutas.
- **Las métricas de suavidad** se calculan sobre todo el intento, no sobre cada movimiento. Con el teclado no significan nada.
- **La tinta real** no se mide con la cámara: se compara el fantasma contra la punta del robot.

## 5. Siguientes pasos

1. **S1 con el pizarrón de 240 × 118:** que el barrido vea los 6 ArUco. Luego repetir S2 y S3 con la zona nueva y las métricas de precisión (las del README son del papel de 3 m).
2. **Robot real, F0 a F6.** Ahí se miden los valores que faltan:
    - `tool_offset` del plumón;
    - `press_depth`;
    - el HSV del azul con la luz del salón;
    - la altura del pizarrón (`bottom_height`) y la distancia del robot a la pared (`scan.wall_distance`).
3. **Commit** del pizarrón de 240 × 118, los 6 ArUco, el barrido calculado y esta guía (sin coautor). Lo anterior ya está en `c6a0078`.
4. **Pendientes de la planeación** que no puedo decidir yo:
    - terapeuta que valide umbrales y duración;
    - nivel MACS de cada niño;
    - modelo de joystick y switch de la cajita;
    - comité de ética, consentimiento y asentimiento;
    - gamificación final.
5. **Mejoras:**
    - J3 continuo;
    - sonidos y colección de figuras para la gamificación;
    - medir la tinta real con la cámara (segmentar el azul);
    - la cajita física.
6. **Ordenar el entorno de Python:** que el `.venv` traiga una SciPy compatible con NumPy 1.26, para quitar el aviso.
