"""El teclado como mando: arma mensajes /joy con el mismo mapeo que el control real.

Para probar en simulación sin el control conectado. Flechas = joystick izquierdo,
espacio = switch (ZR), Enter = A (confirmar), Retroceso = B (cancelar).
Las teclas se leen con Tk, filtrando la auto-repetición de X11 (un soltar seguido de un
presionar en menos de 40 ms es la misma tecla sostenida).
"""

from sensor_msgs.msg import Joy

KEYS_AXES = {'Left': (-1.0, 0.0), 'Right': (1.0, 0.0), 'Up': (0.0, 1.0), 'Down': (0.0, -1.0)}
KEYS_BUTTONS = {'space': 'switch', 'Return': 'confirmar', 'BackSpace': 'cancelar'}


def declare_mapping(node):
    """Mismo mapeo que entrada_joystick (config/mandos/<mando>.yaml)."""
    p = node.declare_parameter
    return {
        'x': p('mando.ejes.x', 0).value, 'x_inv': p('mando.ejes.x_invertido', True).value,
        'y': p('mando.ejes.y', 1).value, 'y_inv': p('mando.ejes.y_invertido', False).value,
        'confirmar': p('mando.botones.confirmar', 1).value,
        'cancelar': p('mando.botones.cancelar', 0).value,
        'switch': p('mando.botones.switch', 8).value,
    }


class KeyboardJoy:
    def __init__(self, root, node, mapping, n_axes=6, n_buttons=14, rate_ms=20):
        self.root, self.node, self.map = root, node, mapping
        self.n_axes, self.n_buttons = n_axes, n_buttons
        self.pub = node.create_publisher(Joy, '/joy', 10)
        self.down = set()
        self._pending = {}
        for key in list(KEYS_AXES) + list(KEYS_BUTTONS):
            root.bind(f'<KeyPress-{key}>', lambda e, k=key: self._press(k))
            root.bind(f'<KeyRelease-{key}>', lambda e, k=key: self._release(k))
        self.rate_ms = rate_ms
        root.after(rate_ms, self._publish)

    def _press(self, key):
        pending = self._pending.pop(key, None)
        if pending is not None:
            self.root.after_cancel(pending)
        self.down.add(key)

    def _release(self, key):
        def really():
            self._pending.pop(key, None)
            self.down.discard(key)
        self._pending[key] = self.root.after(40, really)

    def _publish(self):
        # float(): sin teclas, sum() da el entero 0 y Joy.axes solo acepta flotantes
        x = float(sum(KEYS_AXES[k][0] for k in self.down if k in KEYS_AXES))
        y = float(sum(KEYS_AXES[k][1] for k in self.down if k in KEYS_AXES))
        axes = [0.0] * self.n_axes
        buttons = [0] * self.n_buttons
        m = self.map
        axes[m['x']] = -x if m['x_inv'] else x
        axes[m['y']] = -y if m['y_inv'] else y
        for key, name in KEYS_BUTTONS.items():
            if key in self.down:
                buttons[m[name]] = 1
        msg = Joy()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.axes, msg.buttons = axes, buttons
        self.pub.publish(msg)
        self.root.after(self.rate_ms, self._publish)
