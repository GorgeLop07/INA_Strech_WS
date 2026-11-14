"""Pantalla de pictogramas para el niño (Tkinter).

Muestra la pregunta, las opciones como fichas grandes (colores, iconos de las figuras, la
cuadrícula del mural, flechas) e ilumina la que toca en el barrido. Abajo se ve el mural con
lo que lleva pintado.

Teclado (para probar sin switch físico):
  espacio  switch principal (presionar / soltar)
  n        segundo switch (barrido por pasos)
  a        el adulto elige la opción iluminada (queda registrado como "adulto")
  p / Esc  PARO          r  reanudar
También se puede usar el botón grande con el mouse.
"""

import signal
import queue
import threading
import time
import tkinter as tk

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Empty, String

from muralistas_common.colors import PALETTE, hex_color
from muralistas_common.layout import MuralLayout
from muralistas_common.shapes import FREE_DIRECTIONS, SHAPES
from muralistas_interfaces.msg import ScanState, SwitchEvent

HIGHLIGHT = '#FFD400'
BG = '#1E2430'
TILE_BG = '#2E3646'
TEXT = '#F4F4F4'


class UiRosNode(Node):
    def __init__(self, inbox):
        super().__init__('scanning_ui')
        p = self.declare_parameter
        self.layout = MuralLayout.from_dict({k: p(f'mural.{k}', v).value for k, v in MuralLayout().__dict__.items()})
        self.inbox = inbox
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_raw = self.create_publisher(SwitchEvent, '/hri/switch_raw', 10)
        self.pub_adult = self.create_publisher(Empty, '/hri/adult_select', 10)
        self.pub_stop = self.create_publisher(Bool, '/muralistas/stop', 10)
        self.create_subscription(ScanState, '/hri/scan_state', lambda m: inbox.put(('state', m)), latched)
        self.create_subscription(Image, '/mural/canvas', lambda m: inbox.put(('canvas', m)), latched)
        self.create_subscription(String, '/control/status', lambda m: inbox.put(('status', m.data)), latched)

    def switch(self, switch_id, pressed):
        msg = SwitchEvent()
        msg.stamp = self.get_clock().now().to_msg()
        msg.switch_id = switch_id
        msg.pressed = pressed
        self.pub_raw.publish(msg)


class ScanningUi:
    def __init__(self, node, inbox):
        self.node = node
        self.inbox = inbox
        self.state = None
        self.robot_status = ''
        self.canvas_img = None
        self.root = tk.Tk()
        self.root.title('Pequeños muralistas')
        self.root.configure(bg=BG)
        self.root.geometry('1100x780')
        self.prompt = tk.Label(self.root, text='Conectando...', font=('Sans', 34, 'bold'), fg=TEXT, bg=BG)
        self.prompt.pack(pady=(18, 4))
        self.hint = tk.Label(self.root, text='', font=('Sans', 18), fg=HIGHLIGHT, bg=BG)
        self.hint.pack()
        self.board = tk.Canvas(self.root, width=1060, height=330, bg=BG, highlightthickness=0)
        self.board.pack(pady=8)
        self.summary = tk.Label(self.root, text='', font=('Sans', 18), fg=TEXT, bg=BG)
        self.summary.pack()
        self.mural = tk.Label(self.root, bg=BG)
        self.mural.pack(pady=6)
        bottom = tk.Frame(self.root, bg=BG)
        bottom.pack(fill='x', pady=8)
        self.button = tk.Label(bottom, text='SWITCH  (espacio)', font=('Sans', 22, 'bold'), fg='#1E2430',
                               bg='#7FD46B', width=22, height=2)
        self.button.pack(side='left', padx=30)
        self.button.bind('<ButtonPress-1>', lambda e: self._press(0))
        self.button.bind('<ButtonRelease-1>', lambda e: self._release(0))
        self.status = tk.Label(bottom, text='', font=('Sans', 16), fg=TEXT, bg=BG, justify='left')
        self.status.pack(side='left', padx=20)
        help_text = 'a: elige el adulto   ·   p: PARO   ·   r: reanudar   ·   n: 2º switch'
        tk.Label(self.root, text=help_text, font=('Sans', 12), fg='#9AA3B5', bg=BG).pack()

        self._key_down = {}
        self._pending_release = {}
        self.root.bind('<KeyPress-space>', lambda e: self._key_press('space', 0))
        self.root.bind('<KeyRelease-space>', lambda e: self._key_release('space', 0))
        self.root.bind('<KeyPress-n>', lambda e: self._key_press('n', 1))
        self.root.bind('<KeyRelease-n>', lambda e: self._key_release('n', 1))
        self.root.bind('<KeyPress-a>', lambda e: node.pub_adult.publish(Empty()))
        self.root.bind('<KeyPress-p>', lambda e: node.pub_stop.publish(Bool(data=True)))
        self.root.bind('<Escape>', lambda e: node.pub_stop.publish(Bool(data=True)))
        self.root.bind('<KeyPress-r>', lambda e: node.pub_stop.publish(Bool(data=False)))
        self.root.after(50, self._poll)

    # --- Switch por teclado (sin la auto-repetición de X11) -------------------
    def _key_press(self, key, switch_id):
        pending = self._pending_release.pop(key, None)
        if pending is not None:
            self.root.after_cancel(pending)      # era auto-repetición: sigue presionado
            return
        if not self._key_down.get(key):
            self._key_down[key] = True
            self._press(switch_id)

    def _key_release(self, key, switch_id):
        def really_released():
            self._pending_release.pop(key, None)
            self._key_down[key] = False
            self._release(switch_id)
        self._pending_release[key] = self.root.after(40, really_released)

    def _press(self, switch_id):
        self.button.configure(bg=HIGHLIGHT)
        self.node.switch(switch_id, True)

    def _release(self, switch_id):
        self.button.configure(bg='#7FD46B')
        self.node.switch(switch_id, False)

    # --- Actualización --------------------------------------------------------
    def _poll(self):
        if not rclpy.ok():          # Ctrl+C: ROS se apagó, se cierra la ventana
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            self.root.destroy()
            return
        redraw = False
        try:
            while True:
                kind, payload = self.inbox.get_nowait()
                if kind == 'state':
                    self.state = payload
                    redraw = True
                elif kind == 'canvas':
                    self._set_canvas(payload)
                elif kind == 'status':
                    self.robot_status = payload
                    redraw = True
        except queue.Empty:
            pass
        if redraw:
            self._draw()
        self.root.after(50, self._poll)

    def _set_canvas(self, msg):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        scale = max(1, int(np.ceil(msg.width / 900)))
        img = img[::scale, ::scale]
        header = f'P6 {img.shape[1]} {img.shape[0]} 255 '.encode()
        self.canvas_img = tk.PhotoImage(data=header + img.tobytes(), format='PPM')
        self.mural.configure(image=self.canvas_img)

    def _draw(self):
        s = self.state
        if s is None:
            return
        self.prompt.configure(text=s.prompt)
        self.hint.configure(text=s.target_hint)
        self.summary.configure(text=s.selection_summary)
        status = f'Robot: {self.robot_status}' if self.robot_status else ''
        self.status.configure(text=f'Peldaño {s.level} · modo {s.mode}\n{status}',
                              fg='#FF6B6B' if self.robot_status == 'paro' else TEXT)
        self.prompt.configure(fg='#FF6B6B' if s.stage == 'paro' else TEXT)
        c = self.board
        c.delete('all')
        n = len(s.options)
        if n == 0:
            c.create_text(530, 165, text=s.prompt, fill=TEXT, font=('Sans', 30))
            return
        if s.stage in ('fila', 'columna'):
            self._draw_grid(s)
            return
        cols = min(n, 6 if n > 4 else n)
        rows = int(np.ceil(n / cols))
        w, h = 1040 / cols, min(320 / rows, 300)
        for i, key in enumerate(s.options):
            r, col = divmod(i, cols)
            x0, y0 = 10 + col * w, 5 + r * h
            x1, y1 = x0 + w - 14, y0 + h - 12
            hl = i == s.highlighted
            c.create_rectangle(x0, y0, x1, y1, fill=TILE_BG, outline=HIGHLIGHT if hl else '#444C5C',
                               width=12 if hl else 2)
            self._draw_option(s.stage, key, s.labels[i], x0, y0, x1, y1)

    def _draw_option(self, stage, key, label, x0, y0, x1, y1):
        c = self.board
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        size = min(x1 - x0, y1 - y0) * 0.55
        if stage == 'color' and key in PALETTE:
            c.create_oval(cx - size / 2, cy - size / 2 - 15, cx + size / 2, cy + size / 2 - 15,
                          fill=hex_color(key), outline='')
        elif stage == 'figura' and key in SHAPES:
            for stroke in SHAPES[key]:
                pts = []
                for u, v in stroke:
                    pts += [cx - size / 2 + u * size, cy - 15 + size / 2 - v * size]
                if len(pts) == 2:
                    c.create_oval(pts[0] - 5, pts[1] - 5, pts[0] + 5, pts[1] + 5, fill=TEXT, outline='')
                else:
                    c.create_line(*pts, fill=TEXT, width=5, capstyle='round', smooth=False)
        elif stage == 'direccion' and key in FREE_DIRECTIONS:
            a = np.radians(FREE_DIRECTIONS[key])
            dx, dy = np.cos(a) * size / 2, -np.sin(a) * size / 2
            c.create_line(cx - dx, cy - 15 - dy, cx + dx, cy - 15 + dy, fill=TEXT, width=10, arrow='last',
                          arrowshape=(26, 30, 12))
        else:
            c.create_text(cx, cy - 10, text=label, fill=TEXT, font=('Sans', 30, 'bold'))
            return
        c.create_text(cx, y1 - 22, text=label, fill=TEXT, font=('Sans', 16))

    def _draw_grid(self, s):
        c = self.board
        L = self.node.layout
        cw, ch = 1000 / L.cols, 300 / L.rows
        sel_row = None
        if s.stage == 'columna' and 'fila' in s.selection_summary:
            try:
                sel_row = int(s.selection_summary.split('fila ')[1].split(' ')[0]) - 1
            except (IndexError, ValueError):
                sel_row = None
        for r in range(L.rows):
            for col in range(L.cols):
                x0, y0 = 30 + col * cw, 10 + r * ch
                lit = (s.stage == 'fila' and r == s.highlighted) or \
                      (s.stage == 'columna' and r == sel_row and col == s.highlighted)
                dim = s.stage == 'columna' and r != sel_row
                c.create_rectangle(x0, y0, x0 + cw - 8, y0 + ch - 8,
                                   fill=HIGHLIGHT if lit else ('#262D3A' if dim else TILE_BG),
                                   outline='#555E70', width=2)
                if s.stage == 'columna' and r == sel_row:
                    c.create_text(x0 + cw / 2, y0 + ch / 2, text=str(col + 1), font=('Sans', 26, 'bold'),
                                  fill=BG if lit else TEXT)

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    inbox = queue.Queue()
    node = UiRosNode(inbox)
    def spin():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    spinner = threading.Thread(target=spin, daemon=True)
    spinner.start()
    ui = ScanningUi(node, inbox)
    # Tk corre en el hilo principal: Ctrl+C cierra la ventana en orden en vez de cortar un callback.
    signal.signal(signal.SIGINT, lambda *_: ui.root.after(0, ui.root.destroy))
    try:
        ui.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # un segundo Ctrl+C no interrumpe el cierre
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        time.sleep(0.1)
