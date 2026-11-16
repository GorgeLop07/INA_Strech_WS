"""Pantalla del niño para los modos con joystick (Tkinter).

Dibuja la zona de trabajo como si fuera el pizarrón: la plantilla (gris punteada), lo que el
robot ya pintó (azul, siguiendo la punta real mientras el marcador está apoyado), el fantasma
(azul si es "pegado", punteado si es "levantado"), el cursor con el anillo de J1, el menú
levantado / pegado y las estrellas del último intento.

Teclado:  p / Esc  PARO   ·   r  reanudar
Con teclado:=true, además: flechas = joystick, espacio = switch (ZR), Enter = A, Retroceso = B.
"""

import queue
import signal
import threading
import time
import tkinter as tk

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

from muralistas_common.layout import MuralLayout
from muralistas_interfaces.msg import GhostState

from .keyboard_joy import KeyboardJoy, declare_mapping

BG = '#1E2430'
ZONE_BG = '#F4F2EC'
TEXT = '#F4F4F4'
HIGHLIGHT = '#FFD400'
BLUE = '#1A4DD9'
GHOST_UP = '#7C8BB0'
TEMPLATE = '#9A9A9A'
W, H = 960, 600
MENU_H = 130            # franja de abajo para el menú levantado / pegado (no tapa el pizarrón)


class GhostUiRosNode(Node):
    def __init__(self, inbox):
        super().__init__('pantalla_fantasma')
        p = self.declare_parameter
        self.layout = MuralLayout.from_dict({k: p(f'mural.{k}', v).value for k, v in MuralLayout().__dict__.items()})
        self.keyboard = p('teclado', False).value
        self.mapping = declare_mapping(self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_stop = self.create_publisher(Bool, '/muralistas/stop', 10)
        self.create_subscription(GhostState, '/hri/ghost_state', lambda m: inbox.put(('state', m)), latched)
        self.create_subscription(PointStamped, '/control/tip_board',
                                 lambda m: inbox.put(('tip', (m.point.x, m.point.y))), 10)
        self.create_subscription(Bool, '/control/pen_down', lambda m: inbox.put(('pen', m.data)), latched)
        self.create_subscription(String, '/control/status', lambda m: inbox.put(('status', m.data)), latched)


class GhostUi:
    def __init__(self, node, inbox):
        self.node, self.inbox = node, inbox
        self.state = None
        self.tip, self.pen = None, False
        self.ink = []                  # trazos pintados: lista de listas de (u, v)
        self.status = ''
        self.root = tk.Tk()
        self.root.title('Pequeños muralistas · joystick')
        self.root.configure(bg=BG)
        self.root.geometry('1000x840')
        self.prompt = tk.Label(self.root, text='Conectando...', font=('Sans', 26, 'bold'), fg=TEXT, bg=BG,
                               wraplength=960)
        self.prompt.pack(pady=(14, 2))
        self.level = tk.Label(self.root, text='', font=('Sans', 16), fg=HIGHLIGHT, bg=BG)
        self.level.pack()
        self.canvas = tk.Canvas(self.root, width=W, height=H, bg=BG, highlightthickness=0)
        self.canvas.pack(pady=8)
        self.score = tk.Label(self.root, text='', font=('Sans', 26), fg=HIGHLIGHT, bg=BG)
        self.score.pack()
        self.status_label = tk.Label(self.root, text='', font=('Sans', 14), fg=TEXT, bg=BG)
        self.status_label.pack()
        keys = 'p: PARO · r: reanudar'
        if node.keyboard:
            keys = 'flechas: joystick · espacio: switch · Enter: A · Retroceso: B · ' + keys
            self.joy = KeyboardJoy(self.root, node, node.mapping)
        tk.Label(self.root, text=keys, font=('Sans', 12), fg='#9AA3B5', bg=BG).pack()
        self.root.bind('<KeyPress-p>', lambda e: node.pub_stop.publish(Bool(data=True)))
        self.root.bind('<Escape>', lambda e: node.pub_stop.publish(Bool(data=True)))
        self.root.bind('<KeyPress-r>', lambda e: node.pub_stop.publish(Bool(data=False)))
        u0, v0, u1, v1 = node.layout.work_rect()
        self.zone = (u0, v0, u1, v1)
        margin = 0.04
        board_h = H - MENU_H - 10
        self.scale = min((W - 40) / (u1 - u0 + 2 * margin), board_h / (v1 - v0 + 2 * margin))
        self.ox = (W - (u1 - u0) * self.scale) / 2
        self.oy = (board_h - (v1 - v0) * self.scale) / 2
        self.root.after(40, self._poll)

    def px(self, u, v):
        """Del pizarrón (m) a píxeles: la zona de trabajo llena el lienzo, v hacia arriba."""
        u0, v0, _, v1 = self.zone
        return self.ox + (u - u0) * self.scale, self.oy + (v1 - v) * self.scale

    def _poll(self):
        if not rclpy.ok():
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            self.root.destroy()
            return
        try:
            while True:
                kind, payload = self.inbox.get_nowait()
                if kind == 'state':
                    self.state = payload
                elif kind == 'tip':
                    self.tip = payload
                    if self.pen:
                        if not self.ink or self.ink[-1] is None:
                            self.ink.append([])
                        if self.ink[-1] is not None:
                            self.ink[-1].append(payload)
                elif kind == 'pen':
                    self.pen = payload
                    if not payload:
                        self.ink.append(None)        # separa trazos
                elif kind == 'status':
                    self.status = payload
        except queue.Empty:
            pass
        self._draw()
        self.root.after(40, self._poll)

    def _line(self, pts, **kw):
        coords = []
        for u, v in pts:
            coords += self.px(u, v)
        if len(coords) >= 4:
            self.canvas.create_line(*coords, capstyle='round', joinstyle='round', **kw)

    def _draw(self):
        c = self.canvas
        c.delete('all')
        u0, v0, u1, v1 = self.zone
        x0, y0 = self.px(u0, v1)
        x1, y1 = self.px(u1, v0)
        c.create_rectangle(x0, y0, x1, y1, fill=ZONE_BG, outline='#555E70', width=2)
        s = self.state
        # La plantilla abajo y la tinta encima: que se vea lo que pintó el robot sobre la guía
        if s is not None and s.template_points:
            self._line([(p.x, p.y) for p in s.template_points], fill=TEMPLATE, width=10, dash=(12, 10))
        for stroke in self.ink:
            if stroke:
                self._line(stroke, fill=BLUE, width=6)
        self.status_label.configure(text=f'Robot: {self.status}' if self.status else '',
                                    fg='#FF6B6B' if self.status == 'paro' else TEXT)
        if s is None:
            return
        self.prompt.configure(text=s.prompt, fg='#FF6B6B' if s.stage == 'paro' else TEXT)
        self.level.configure(text=s.level_text)
        if s.stars >= 0:
            self.score.configure(text=('★' * s.stars + '☆' * (3 - s.stars)) + '   ' + s.score_text)
        ghost = [(p.x, p.y) for p in s.ghost]
        if s.stage in ('trazando', 'armando', 'esperando') and len(ghost) > 1:
            if s.pen_down:
                extra = {'stipple': 'gray50'} if s.stage == 'esperando' else {}
                self._line(ghost, fill=BLUE, width=8, **extra)
            else:
                self._line(ghost, fill=GHOST_UP, width=5, dash=(8, 8))
        if self.tip is not None:
            tx, ty = self.px(*self.tip)
            c.create_oval(tx - 7, ty - 7, tx + 7, ty + 7, fill='#E04040' if self.pen else '', outline='#E04040',
                          width=3)
        if s.stage in ('trazando', 'armando', 'menu'):
            # En el menú el joystick elige, no dibuja: el cursor se queda en la punta
            if s.stage == 'menu' and self.tip is not None:
                cx, cy = self.px(*self.tip)
            else:
                cx, cy = self.px(s.cursor.x, s.cursor.y)
            color = HIGHLIGHT if s.stage != 'armando' else '#7FD46B'
            c.create_oval(cx - 14, cy - 14, cx + 14, cy + 14, outline='#333', width=3, fill=color)
            if s.dwell_progress > 0:
                c.create_arc(cx - 26, cy - 26, cx + 26, cy + 26, start=90, extent=-360 * s.dwell_progress,
                             style='arc', outline='#7FD46B', width=8)
        if s.stage == 'menu':
            self._draw_menu(s.menu_highlight)

    def _draw_menu(self, highlight):
        c = self.canvas
        for i, (key, label, sub) in enumerate([('levantado', 'Levantado', '← mover sin pintar'),
                                               ('pegado', 'Pegado', 'pintar →')]):
            x0 = 40 + i * (W / 2)
            x1 = x0 + W / 2 - 80
            lit = key == highlight
            c.create_rectangle(x0, H - MENU_H + 10, x1, H - 10, fill='#2E3646', outline=HIGHLIGHT if lit else '#444C5C',
                               width=12 if lit else 2)
            c.create_text((x0 + x1) / 2, H - MENU_H + 50, text=label, fill=TEXT, font=('Sans', 30, 'bold'))
            c.create_text((x0 + x1) / 2, H - 35, text=sub, fill=TEXT, font=('Sans', 16))

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    inbox = queue.Queue()
    node = GhostUiRosNode(inbox)

    def spin():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    threading.Thread(target=spin, daemon=True).start()
    ui = GhostUi(node, inbox)
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
