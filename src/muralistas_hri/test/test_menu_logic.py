from muralistas_hri.menu_logic import MenuConfig, ScanningMenu


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def select(menu, key):
    """Avanza el barrido hasta `key` y presiona."""
    for _ in range(len(menu.options) + 1):
        if menu.options[menu.highlighted] == key:
            return menu.press()
        menu.advance()
    raise AssertionError(f'{key} no está en {menu.options}')


def test_peldano_3_figura_completa():
    menu = ScanningMenu(MenuConfig(level=3), clock=FakeClock())
    assert menu.stage == 'color'
    select(menu, 'azul')
    assert menu.stage == 'figura'
    select(menu, 'circulo')
    select(menu, '1')            # fila
    actions, decisions = select(menu, '2')   # columna
    assert [a.kind for a in actions] == ['shape']
    a = actions[0]
    assert (a.color, a.shape, a.row, a.col) == ('azul', 'circulo', 1, 2)
    assert decisions[0].source == 'nino'
    assert menu.waiting and menu.stage == 'pintando'
    menu.robot_busy(True)
    menu.robot_busy(False)
    assert menu.stage == 'color' and not menu.waiting


def test_peldano_1_una_presion_pinta_y_registra_lo_automatico():
    menu = ScanningMenu(MenuConfig(level=1), clock=FakeClock())
    actions, decisions = menu.press()
    assert actions[0].kind == 'shape'
    sources = [d.source for d in decisions]
    assert sources.count('nino') == 1 and sources.count('auto') == 3


def test_mientras_pinta_se_ignoran_presiones():
    menu = ScanningMenu(MenuConfig(level=1), clock=FakeClock())
    menu.press()
    assert menu.press() == ([], [])


def test_trazo_libre_toggle():
    menu = ScanningMenu(MenuConfig(mode='libre'), clock=FakeClock())
    select(menu, 'rojo')
    select(menu, '0')
    actions, _ = select(menu, '1')
    assert actions[0].kind == 'move_to_cell'
    menu.robot_busy(True)
    menu.robot_busy(False)
    assert menu.stage == 'direccion'
    actions, _ = select(menu, 'arriba')
    assert actions[0].kind == 'free_start' and actions[0].direction_deg == 90.0
    assert menu.stage == 'dibujando'
    actions, _ = menu.press()          # segunda presión: alto
    assert actions[0].kind == 'free_stop'
    assert menu.stage == 'direccion'
    actions, _ = select(menu, 'levantar')
    assert actions[0].kind == 'pen_up'


def test_trazo_libre_mantener_para_al_soltar():
    menu = ScanningMenu(MenuConfig(mode='libre', free_activation='hold'), clock=FakeClock())
    select(menu, 'rojo')
    select(menu, '0')
    select(menu, '0')
    menu.robot_busy(True)
    menu.robot_busy(False)
    select(menu, 'derecha')
    assert menu.press() == ([], [])    # presionar de nuevo no hace nada en modo mantener
    actions, _ = menu.release()
    assert actions[0].kind == 'free_stop'


def test_pausa_tras_vueltas_y_reanudar_sin_elegir():
    menu = ScanningMenu(MenuConfig(level=3), clock=FakeClock())
    menu.pause()
    assert menu.highlighted == -1
    actions, decisions = menu.press()
    assert actions == [] and decisions == [] and menu.highlighted == 0


def test_adulto_queda_registrado():
    menu = ScanningMenu(MenuConfig(level=2), clock=FakeClock())
    _, decisions = menu.press(source='adulto')
    assert decisions[0].source == 'adulto'
