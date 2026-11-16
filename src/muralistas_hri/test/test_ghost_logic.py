import math

from muralistas_common.templates import path_length
from muralistas_hri.ghost_logic import GhostConfig, GhostInteraction


def _drive(g, t, path, dt=0.02, deflection=(0.5, 0.0)):
    """Mueve el cursor por `path` (lista de puntos) a un punto por ciclo."""
    actions = []
    for p in path:
        t += dt
        actions += g.update(t, p, deflection)
    return t, actions


def _line(a, b, n=40):
    return [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(1, n + 1)]


def test_j2_menu_pegado_trazar_y_confirmar_con_el_switch():
    g = GhostInteraction(GhostConfig(interaction='J2'))
    acts = g.start((0.5, 0.5), 0.0)
    assert g.stage == 'menu' and acts[0] == ('place_cursor', (0.5, 0.5))
    g.update(0.1, (0.5, 0.5), (0.9, 0.0))                     # joystick a la derecha: pegado
    assert g.highlight == 'pegado'
    acts = g.switch(False, 0.2, 0.3)
    assert g.stage == 'trazando' and g.pen_state == 'pegado'
    t, _ = _drive(g, 0.3, _line((0.5, 0.5), (0.6, 0.5)))
    acts = g.switch(False, 0.2, t)                           # presión corta: confirmar
    ghost = [a for a in acts if a[0] == 'ghost'][0]
    assert ghost[2] is True and ghost[3] is False             # pinta y deja el pincel apoyado
    assert math.isclose(ghost[1][0][0], 0.5) and ghost[1][-1][0] > 0.59
    assert g.stage == 'esperando'
    g.robot_busy(True, (0.55, 0.5), t + 0.5)
    acts = g.robot_busy(False, (0.6, 0.5), t + 3.0)
    assert g.stage == 'menu' and acts[0] == ('place_cursor', (0.6, 0.5))


def test_j2_presion_larga_borra_y_fantasma_corto_no_se_confirma():
    g = GhostInteraction(GhostConfig(interaction='J2'))
    g.start((0.5, 0.5), 0.0)
    g.update(0.1, (0.5, 0.5), (-0.9, 0.0))                    # izquierda: levantado
    g.switch(False, 0.2, 0.2)
    assert g.pen_state == 'levantado'
    t, _ = _drive(g, 0.2, _line((0.5, 0.5), (0.51, 0.5), n=5))
    assert g.switch(False, 0.2, t) == []                     # 1 cm: muy corto
    t, _ = _drive(g, t, _line((0.51, 0.5), (0.6, 0.5)))
    acts = g.switch(False, 1.5, t)                           # presión larga
    assert ('place_cursor', (0.5, 0.5)) in acts and len(g.ghost) == 1


def test_j1_confirma_quieto_y_un_movimiento_cancela_el_arranque():
    g = GhostInteraction(GhostConfig(interaction='J1', dwell=1.0, start_delay=1.0))
    g.start((0.5, 0.5), 0.0)
    g.update(0.1, (0.5, 0.5), (0.9, 0.0))                     # ilumina pegado
    t = 0.1
    for _ in range(60):                                       # suelta el joystick: quieto 1.2 s
        t += 0.02
        g.update(t, (0.5, 0.5), (0.0, 0.0))
    assert g.stage == 'trazando' and g.pen_state == 'pegado'
    t, _ = _drive(g, t, _line((0.5, 0.5), (0.6, 0.5)))
    for _ in range(55):                                       # quieto 1.1 s al final
        t += 0.02
        g.update(t, (0.6, 0.5), (0.0, 0.0))
    assert g.stage == 'armando'
    g.update(t + 0.1, (0.62, 0.5))                            # se movió: cancela
    assert g.stage == 'trazando'
    for _ in range(60):
        t += 0.02
        g.update(t, (0.62, 0.5))
    assert g.stage == 'armando'
    acts = g.update(t + 1.05, (0.62, 0.5))
    assert any(a[0] == 'ghost' for a in acts) and g.stage == 'esperando'


def test_j1_quieto_en_el_inicio_borra():
    g = GhostInteraction(GhostConfig(interaction='J1', dwell=1.0))
    g.start((0.5, 0.5), 0.0)
    g._choose('pegado', 0.0)
    t, _ = _drive(g, 0.0, _line((0.5, 0.5), (0.6, 0.5)) + _line((0.6, 0.5), (0.505, 0.5)))
    acts = []
    for _ in range(60):
        t += 0.02
        acts += g.update(t, (0.505, 0.5))
    # Se borró: el cursor regresa al inicio y el fantasma vuelve a empezar ahí
    assert ('place_cursor', (0.5, 0.5)) in acts and g.stage == 'trazando'
    assert g.ghost[0] == (0.5, 0.5) and path_length(g.ghost) < 0.01


def test_rehabilitacion_iman_calificacion_y_pinta_el_fantasma():
    tmpl = [(0.5, 0.5), (0.8, 0.5)]
    g = GhostInteraction(GhostConfig(interaction='J3', mode='rehabilitacion', template=tmpl,
                                     template_name='linea', magnet=0.5))
    acts = g.start((0.4, 0.4), 0.0)
    assert acts == [('ghost', [(0.5, 0.5)], False, False)]   # primero se desplaza al inicio
    g.robot_busy(True, (0.45, 0.45), 0.5)
    acts = g.robot_busy(False, (0.5, 0.5), 2.0)
    assert g.stage == 'trazando' and acts == [('place_cursor', (0.5, 0.5))]
    path = [(0.5 + 0.3 * i / 60, 0.5 + 0.01) for i in range(1, 61)]   # 1 cm arriba de la línea
    t, _ = _drive(g, 2.0, path)
    assert abs(g.ghost[-1][1] - 0.505) < 1e-6               # con imán 0.5 se ve a medio camino
    acts = g.switch(False, 0.2, t)
    att = [a for a in acts if a[0] == 'attempt'][0][1]
    assert abs(att['mean_error'] - 0.01) < 0.002 and att['coverage'] > 0.9 and att['stars'] >= 2
    ghost = [a for a in acts if a[0] == 'ghost'][0]
    assert ghost[2] is True and ghost[3] is True              # pinta y levanta al final
    g.robot_busy(True, None, t + 0.5)
    acts = g.robot_busy(False, (0.8, 0.5), t + 5.0)
    assert acts == [('ghost', [(0.5, 0.5)], False, False)]   # siguiente intento: de vuelta al inicio
