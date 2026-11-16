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
    assert menu.stage == 'figura'          # sin menú de colores: el marcador es azul
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
    assert menu.stage == 'figura' and not menu.waiting


def test_peldano_1_una_presion_pinta_y_registra_lo_automatico():
    menu = ScanningMenu(MenuConfig(level=1), clock=FakeClock())
    actions, decisions = menu.press()
    assert actions[0].kind == 'shape'
    sources = [d.source for d in decisions]
    assert sources.count('nino') == 1 and sources.count('auto') == 2   # figura y casilla


def test_mientras_pinta_se_ignoran_presiones():
    menu = ScanningMenu(MenuConfig(level=1), clock=FakeClock())
    menu.press()
    assert menu.press() == ([], [])


def test_trazo_libre_toggle():
    menu = ScanningMenu(MenuConfig(mode='libre'), clock=FakeClock())
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


def test_peldano_2_elige_figura_y_la_casilla_es_automatica():
    menu = ScanningMenu(MenuConfig(level=2), clock=FakeClock())
    assert menu.stage == 'figura'
    actions, decisions = select(menu, 'sol')
    assert actions[0].kind == 'shape' and actions[0].shape == 'sol' and actions[0].color == 'azul'
    assert [d.source for d in decisions] == ['nino', 'auto']


def test_peldano_4_acepta_el_formato_viejo_con_color():
    menu = ScanningMenu(MenuConfig(level=4, template=['sol:amarillo:0:0', 'ola:2:1']), clock=FakeClock())
    assert 'sol' in menu.target_hint() and 'amarillo' not in menu.target_hint()


def _rehab_menu(clock):
    # Cuadrado de 0.30 m con esquina en (1.0, 0.3): tramos derecha, arriba, izquierda, abajo
    pts = [(1.0, 0.3), (1.3, 0.3), (1.3, 0.6), (1.0, 0.6), (1.0, 0.3)]
    return ScanningMenu(MenuConfig(mode='rehabilitacion', rehab_points=pts, rehab_name='cuadrado',
                                   switch_speed=0.02, overshoot=0.05), clock=clock)


def test_rehabilitacion_switch_mide_el_paro_y_califica():
    clock = FakeClock()
    menu = _rehab_menu(clock)
    assert menu.stage == 'inicio'
    actions, _ = select(menu, 'empezar')
    assert actions[0].kind == 'move_to_point' and actions[0].point == (1.0, 0.3)
    menu.robot_busy(True, tip=(1.0, 0.3))
    menu.robot_busy(False, tip=(1.0, 0.3))
    assert menu.stage == 'direccion' and 'levantar' not in menu.options
    # Una dirección equivocada se registra y se vuelve a preguntar
    actions, decisions = select(menu, 'arriba')
    assert actions == [] and 'equivocada' in decisions[0].choice and menu.stage == 'direccion'
    tips = [(1.31, 0.3), (1.31, 0.585), (0.99, 0.585), (0.99, 0.3)]   # +1 cm, -1.5 cm, +2 cm (con el desfase), ...
    for direction, tip in zip(['derecha', 'arriba', 'izquierda', 'abajo'], tips):
        actions, _ = select(menu, direction)
        assert actions[0].kind == 'free_start' and actions[0].speed == 0.02
        assert abs(actions[0].length - 0.35) < 1e-9          # tramo de 0.30 + 0.05 de margen
        menu.robot_busy(True, tip=tip)
        actions, _ = menu.press()                            # el niño para
        assert actions[0].kind == 'free_stop'
        out = menu.robot_busy(False, tip=tip)
    assert out and out[0].kind == 'pen_up'                   # fin del intento
    att = menu.attempts[-1]
    assert att['wrong_directions'] == 1 and len(att['stop_errors']) == 4
    assert len(att['reaction_times']) == 4 and len(att['choice_times']) == 4
    assert abs(att['stop_errors'][0] - 0.01) < 1e-9          # se pasó 1 cm en el primer tramo
    assert att['stars'] >= 1


def test_rehabilitacion_switch_si_no_para_cuenta_como_no_paro():
    clock = FakeClock()
    menu = _rehab_menu(clock)
    select(menu, 'empezar')
    menu.robot_busy(True, tip=(1.0, 0.3))
    menu.robot_busy(False, tip=(1.0, 0.3))
    select(menu, 'derecha')
    menu.robot_busy(True, tip=(1.2, 0.3))
    menu.robot_busy(False, tip=(1.35, 0.3))                  # llegó al final del tramo alargado
    assert menu.attempt['reached_end'] == 1 and menu.stage == 'direccion'
