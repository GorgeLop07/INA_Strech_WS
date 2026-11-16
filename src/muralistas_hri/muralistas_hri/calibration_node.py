"""Calibración del niño (pantalla Tkinter): pruebas C1 a C5 y perfil sugerido (C6, C7).

  C1 switch: 10 luces a intervalos al azar de 2 a 5 s; presionar cuando se prende.
  C2 reposo: 10 s con la mano en el joystick, sin moverlo.
  C3 alcance: 8 direcciones x 2, en orden al azar; empujar hacia el objetivo y regresar.
  C4 círculo: seguir un punto que da 3 vueltas.
  C5 dos manos: sostener el joystick a un lado y presionar el switch con la otra mano (3 intentos).

El adulto controla el ritmo: Enter empieza cada prueba, s la salta (p. ej. sin joystick), Esc sale.
Para probar sin el niño: espacio = switch; con teclado:=true, flechas = joystick.
Guarda datos/calibracion_<usuario>_<fecha>.json (todo lo medido) y
datos/perfil_<usuario>_<fecha>.yaml (para el launch: perfil:=esa ruta).
"""

import json
import math
import os
import random
import signal
import threading
import time
import tkinter as tk
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy

from . import calibration_logic as cal
from .keyboard_joy import KeyboardJoy, declare_mapping

def _clean(o):
    """Para JSON: NaN -> null, numpy -> listas y números de Python."""
    if isinstance(o, dict):
        return {str(k): _clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_clean(v) for v in o]
    if isinstance(o, np.ndarray):
        return _clean(o.tolist())
    if isinstance(o, (float, np.floating)):
        return None if math.isnan(o) else float(o)
    if isinstance(o, np.integer):
        return int(o)
    return o


BG, TEXT, HIGHLIGHT, GREEN = '#1E2430', '#F4F4F4', '#FFD400', '#7FD46B'
CW, CH, R = 760, 480, 200          # lienzo y radio (px) de una deflexión de 1.0


class CalibrationRosNode(Node):
    def __init__(self):
        super().__init__('calibracion')
        p = self.declare_parameter
        self.mapping = declare_mapping(self)
        self.user_id = p('user_id', 'nino_01').value
        self.macs = p('macs', 0).value
        self.keyboard = p('teclado', False).value
        self.fs = p('frecuencia', 100.0).value
        default_dir = Path(os.environ.get('INA_STRECH_WS', Path.cwd())) / 'datos'
        self.out_dir = Path(p('output_dir', str(default_dir)).value)
        self.lock = threading.Lock()
        self.defl = (0.0, 0.0)
        self.switch_down = False
        self.switch_events = []          # (t, pressed)
        self.recording = None            # lista de (t, x, y) mientras se graba
        self.create_subscription(Joy, '/joy', self._on_joy, 50)
        self.create_timer(1.0 / self.fs, self._sample)

    def _on_joy(self, msg):
        m = self.mapping
        ax = lambda i, inv: (-msg.axes[i] if inv else msg.axes[i]) if i < len(msg.axes) else 0.0  # noqa: E731
        self.defl = (ax(m['x'], m['x_inv']), ax(m['y'], m['y_inv']))
        b = m['switch']
        pressed = b < len(msg.buttons) and bool(msg.buttons[b])
        if pressed != self.switch_down:
            self.switch(pressed)

    def switch(self, pressed):
        with self.lock:
            self.switch_down = pressed
            self.switch_events.append((time.monotonic(), pressed))

    def _sample(self):
        with self.lock:
            if self.recording is not None:
                self.recording.append((time.monotonic(), self.defl[0], self.defl[1]))

    def start_recording(self):
        with self.lock:
            self.recording = []

    def stop_recording(self):
        with self.lock:
            rec, self.recording = self.recording or [], None
        return np.array([[x, y] for _, x, y in rec]) if rec else np.zeros((0, 2))

    def events_since(self, t0):
        with self.lock:
            return [e for e in self.switch_events if e[0] >= t0]


class CalibrationUi:
    def __init__(self, node):
        self.node = node
        self.rng = random.Random()
        self.root = tk.Tk()
        self.root.title('Pequeños muralistas · calibración')
        self.root.configure(bg=BG)
        self.root.geometry('900x720')
        self.title = tk.Label(self.root, text='Calibración', font=('Sans', 28, 'bold'), fg=TEXT, bg=BG)
        self.title.pack(pady=(14, 2))
        self.info = tk.Label(self.root, text='', font=('Sans', 18), fg=TEXT, bg=BG, wraplength=860, justify='center')
        self.info.pack(pady=4)
        self.canvas = tk.Canvas(self.root, width=CW, height=CH, bg=BG, highlightthickness=0)
        self.canvas.pack(pady=6)
        keys = 'Enter: empezar / siguiente · s: saltar prueba · Esc: salir · espacio: switch'
        if node.keyboard:
            keys += ' · flechas: joystick'
            self.joy = KeyboardJoy(self.root, node, node.mapping)
        tk.Label(self.root, text=keys, font=('Sans', 12), fg='#9AA3B5', bg=BG).pack()
        self.root.bind('<Return>', lambda e: self._next())
        self.root.bind('<KeyPress-s>', lambda e: self._skip())
        self.root.bind('<Escape>', lambda e: self.root.destroy())
        if not node.keyboard:         # con teclado:=true el espacio ya es el switch de /joy
            self.root.bind('<KeyPress-space>', lambda e: self._space(True))
            self.root.bind('<KeyRelease-space>', lambda e: self._space(False))
        self._space_down = False
        self.results = {}
        self.tests = ['c1', 'c2', 'c3', 'c4', 'c5']
        self.test_i = -1
        self.running = None
        self._show_intro()
        self.root.after(30, self._loop)

    # --- Navegación -----------------------------------------------------------
    def _space(self, pressed):
        if pressed != self._space_down:
            self._space_down = pressed
            self.node.switch(pressed)

    def _show_intro(self):
        self.title.configure(text='Calibración')
        self.info.configure(text='Son 5 pruebas cortas (unos 3 a 4 minutos). El adulto presiona Enter para '
                                 'empezar cada una. Si el niño no usa joystick, salta C2 a C5 con la tecla s.')

    def _next(self):
        if self.running is not None:
            return
        self.test_i += 1
        if self.test_i >= len(self.tests):
            return self._finish()
        getattr(self, f'_start_{self.tests[self.test_i]}')()

    def _skip(self):
        """Salta la prueba en curso o, si no hay ninguna corriendo, la siguiente."""
        if self.running is not None:
            self.node.stop_recording()
            self.running = None
        else:
            self.test_i += 1
            if self.test_i >= len(self.tests):
                return self._finish()
        name = self.tests[self.test_i]
        self.results.setdefault(name, None)
        self.info.configure(text=f'{name.upper()} saltada. Enter para la siguiente.')

    def _loop(self):
        if not rclpy.ok():
            self.root.destroy()
            return
        if self.running is not None:
            self.running()
        self.root.after(30, self._loop)

    # --- Dibujo ------------------------------------------------------------------
    def _xy(self, x, y):
        return CW / 2 + x * R, CH / 2 - y * R

    def _cursor(self, color=HIGHLIGHT):
        x, y = self._xy(*self.node.defl)
        self.canvas.create_oval(x - 14, y - 14, x + 14, y + 14, fill=color, outline='#333', width=3)

    def _frame(self):
        c = self.canvas
        c.delete('all')
        c.create_oval(CW / 2 - R, CH / 2 - R, CW / 2 + R, CH / 2 + R, outline='#444C5C', width=2)

    # --- C1 switch ---------------------------------------------------------------
    def _start_c1(self):
        self.title.configure(text='C1 · Switch')
        self.info.configure(text='Presiona el switch cuando se prenda la luz verde.')
        self.c1 = {'n': 10, 'i': 0, 'trials': [], 'on_at': None, 'off_until': time.monotonic() + 2.0,
                   't0': time.monotonic()}
        self.running = self._run_c1

    def _run_c1(self):
        s, now, c = self.c1, time.monotonic(), self.canvas
        c.delete('all')
        lit = s['on_at'] is not None
        c.create_oval(CW / 2 - 120, CH / 2 - 120, CW / 2 + 120, CH / 2 + 120, fill=GREEN if lit else '#2E3646',
                      outline='')
        c.create_text(CW / 2, CH - 20, text=f'{s["i"]} de {s["n"]}', fill=TEXT, font=('Sans', 14))
        if not lit and now >= s['off_until']:
            s['on_at'] = now
        elif lit:
            presses = [e for e in self.node.events_since(s['on_at']) if e[1]]
            if presses or now - s['on_at'] > 5.0:
                s['trials'].append({'on': s['on_at'], 'end': now + 1.5})
                s['i'] += 1
                s['on_at'] = None
                s['off_until'] = now + self.rng.uniform(2.0, 5.0)
                if s['i'] >= s['n']:
                    self.running = None
                    self.root.after(1600, self._finish_c1)

    def _finish_c1(self):
        events = self.node.events_since(self.c1['t0'])
        trials, used = [], set()
        trials_c1 = self.c1['trials']
        for i, tr in enumerate(trials_c1):
            # Ventana de la luz: hasta 5 s o hasta que se prende la siguiente, lo que pase primero
            end = min(tr['on'] + 5.0, trials_c1[i + 1]['on'] if i + 1 < len(trials_c1) else tr['on'] + 5.0)
            win = [e for e in events if tr['on'] <= e[0] < end]
            presses = [e[0] for e in win if e[1]]
            releases = [e[0] for e in events if not e[1] and presses and e[0] > presses[0]]
            rt = presses[0] - tr['on'] if presses else None
            hold = releases[0] - presses[0] if presses and releases else None
            gaps = [b - a for a, b in zip(presses[:-1], presses[1:])]
            trials.append({'rt': rt, 'hold': hold, 'bounce': min(gaps) if gaps else None})
            used.update(presses)
        false_presses = sum(1 for e in events if e[1] and e[0] not in used)
        self.results['c1'] = cal.switch_results(trials, false_presses)
        r = self.results['c1']
        rt = f'{r["rt_mediana_s"]:.2f} s' if r['rt_mediana_s'] is not None else 'sin respuesta'
        self.info.configure(text=f'C1 lista: reacción mediana {rt}, sin respuesta {r["sin_respuesta"]}, '
                                 f'presiones de más {r["presiones_de_mas"]}. Enter para la siguiente.')

    # --- C2 reposo -----------------------------------------------------------------
    def _start_c2(self):
        self.title.configure(text='C2 · Reposo')
        self.info.configure(text='Deja la mano en el joystick, sin moverlo, 10 segundos.')
        self.node.start_recording()
        self.c_end = time.monotonic() + 10.0
        self.running = self._run_c2

    def _run_c2(self):
        self._frame()
        self._cursor()
        left = self.c_end - time.monotonic()
        self.canvas.create_text(CW / 2, 30, text=f'{max(0, left):.0f} s', fill=TEXT, font=('Sans', 22))
        if left <= 0:
            self.running = None
            self.results['c2'] = cal.rest_results(self.node.stop_recording())
            r = self.results['c2']
            self.info.configure(text=f'C2 lista: temblor {r["temblor"]:.3f}, deriva {r["deriva"]:.3f} '
                                     f'-> zona muerta {r["zona_muerta"]:.2f}. Enter para la siguiente.')

    # --- C3 alcance --------------------------------------------------------------------
    def _start_c3(self):
        self.title.configure(text='C3 · Alcance')
        self.info.configure(text='Empuja el joystick hacia la estrella y luego suéltalo.')
        order = cal.DIRECTIONS * 2
        self.rng.shuffle(order)
        self.c3 = {'order': order, 'i': 0, 'trials': [], 't': None}
        self.running = self._run_c3

    def _run_c3(self):
        s, now = self.c3, time.monotonic()
        if s['t'] is None:
            s['t'] = now
            self.node.start_recording()
        elapsed = now - s['t']
        d = s['order'][s['i']]
        self._frame()
        if elapsed < 4.0:
            a = math.radians(d)
            tx, ty = self._xy(0.8 * math.cos(a), 0.8 * math.sin(a))
            self.canvas.create_text(tx, ty, text='★', fill=HIGHLIGHT, font=('Sans', 48))
        else:
            self.canvas.create_text(CW / 2, 30, text='Suelta el joystick', fill=TEXT, font=('Sans', 20))
        self._cursor()
        self.canvas.create_text(CW / 2, CH - 20, text=f'{s["i"] + 1} de {len(s["order"])}', fill=TEXT,
                                font=('Sans', 14))
        if elapsed >= 6.0:
            s['trials'].append((d, self.node.stop_recording()))
            s['i'] += 1
            s['t'] = None
            if s['i'] >= len(s['order']):
                self.running = None
                dz = (self.results.get('c2') or {}).get('zona_muerta', 0.10)
                self.results['c3'] = cal.reach_results(s['trials'], self.node.fs, dz)
                self.results['c3_crudo'] = [(dd, xy.round(4).tolist()) for dd, xy in s['trials']]
                r = self.results['c3']
                self.info.configure(text=f'C3 lista: {r["direcciones_cortas"]} direcciones con alcance menor a 0.4, '
                                         f'atorado en {r["atorado"]}, LDLJ {r["ldlj"]:.2f}. Enter para la siguiente.')

    # --- C4 círculo -----------------------------------------------------------------------
    def _start_c4(self):
        self.title.configure(text='C4 · Círculo')
        self.info.configure(text='Sigue el punto amarillo con el joystick: da 3 vueltas.')
        self.node.start_recording()
        self.c_t0 = time.monotonic()
        self.running = self._run_c4

    def _run_c4(self):
        t = time.monotonic() - self.c_t0
        self._frame()
        a = -math.pi / 2 + 2 * math.pi * t / 8.0
        gx, gy = self._xy(0.7 * math.cos(a), 0.7 * math.sin(a))
        self.canvas.create_oval(CW / 2 - 0.7 * R, CH / 2 - 0.7 * R, CW / 2 + 0.7 * R, CH / 2 + 0.7 * R,
                                outline='#9A9A9A', width=6, dash=(10, 8))
        self.canvas.create_oval(gx - 18, gy - 18, gx + 18, gy + 18, fill=HIGHLIGHT, outline='')
        self._cursor(color=GREEN)
        if t >= 24.0:
            self.running = None
            xy = self.node.stop_recording()
            self.results['c4'] = cal.circle_results(xy, self.node.fs)
            self.results['c4_crudo'] = xy.round(4).tolist()
            r = self.results['c4']
            self.info.configure(text=f'C4 lista: error {100 * r["error_relativo"]:.0f}% del radio, '
                                     f'LDLJ {r["ldlj"]:.2f}. Enter para la siguiente.')

    # --- C5 dos manos ----------------------------------------------------------------------
    def _start_c5(self):
        self.title.configure(text='C5 · Dos manos')
        self.info.configure(text='Empuja el joystick a un lado y, sin soltarlo, presiona el switch con la otra mano.')
        self.c5 = {'i': 0, 'ok': [], 't': None, 'pushed_at': None}
        self.running = self._run_c5

    def _run_c5(self):
        s, now = self.c5, time.monotonic()
        if s['t'] is None:
            s['t'], s['pushed_at'] = now, None
        self._frame()
        self._cursor()
        pushed = math.hypot(*self.node.defl) > 0.5
        if pushed and s['pushed_at'] is None:
            s['pushed_at'] = now
        ok = False
        if s['pushed_at'] is not None:
            presses = [e for e in self.node.events_since(s['pushed_at']) if e[1]]
            ok = bool(presses) and presses[0][0] - s['pushed_at'] <= 3.0 and pushed
        self.canvas.create_text(CW / 2, CH - 20, text=f'Intento {s["i"] + 1} de 3', fill=TEXT, font=('Sans', 14))
        if ok or now - s['t'] > 8.0:
            s['ok'].append(ok)
            s['i'] += 1
            s['t'] = None
            if s['i'] >= 3:
                self.running = None
                self.results['c5'] = {'intentos': s['ok'], 'pasa': sum(s['ok']) >= 2}
                self.info.configure(text=f'C5 lista: {sum(s["ok"])} de 3. Enter para ver el resultado.')

    # --- Resultado ---------------------------------------------------------------------------
    def _finish(self):
        r = self.results
        two = (r.get('c5') or {}).get('pasa', False)
        decision = cal.assign(r.get('c1'), r.get('c2'), r.get('c3'), r.get('c4'), two, self.node.macs)
        profile = cal.suggested_profile(self.node.user_id, r.get('c1'), r.get('c2'), r.get('c3'), r.get('c4'), decision)
        self.node.out_dir.mkdir(parents=True, exist_ok=True)
        stamp = f'{datetime.now():%Y%m%d_%H%M%S}'
        jpath = self.node.out_dir / f'calibracion_{self.node.user_id}_{stamp}.json'
        ppath = self.node.out_dir / f'perfil_{self.node.user_id}_{stamp}.yaml'
        jpath.write_text(json.dumps(_clean({**r, 'decision': decision, 'macs': self.node.macs}),
                                    indent=1, ensure_ascii=False))
        with open(ppath, 'w') as f:
            f.write(f'# Perfil sugerido por la calibración del {stamp}. La terapeuta lo revisa antes de usarlo.\n')
            yaml.safe_dump(profile, f, allow_unicode=True, sort_keys=False)
        self.title.configure(text=f'Resultado: modo {decision["interaccion"]}, asistencia {decision["asistencia"]}')
        macs = ''
        if decision['sugerencia_macs']:
            macs = f' · MACS sugiere {decision["sugerencia_macs"]}' + ('' if decision['coincide_con_macs'] else ' (no coincide: revisar)')
        self.info.configure(text=f'{decision["razon"]}{macs}\n\nPerfil: {ppath}\nDatos: {jpath}')
        self.canvas.delete('all')
        self.node.get_logger().info(f'Calibración: {decision}. Perfil en {ppath}')

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = CalibrationRosNode()

    def spin():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    threading.Thread(target=spin, daemon=True).start()
    ui = CalibrationUi(node)
    signal.signal(signal.SIGINT, lambda *_: ui.root.after(0, ui.root.destroy))
    try:
        ui.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        time.sleep(0.1)
