from setuptools import setup

package_name = 'muralistas_hri'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robotics for Inclusion Workshop',
    maintainer_email='jorgelolpez07@gmail.com',
    description='Interacción: filtro del switch, barrido por peldaños, pantalla de pictogramas y registro de decisiones.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'switch_filter = muralistas_hri.switch_filter_node:main',
            'scanning_menu = muralistas_hri.scanning_menu_node:main',
            'scanning_ui = muralistas_hri.scanning_ui_node:main',
            'decision_logger = muralistas_hri.decision_logger_node:main',
            'entrada_joystick = muralistas_hri.joystick_input_node:main',
            'fantasma = muralistas_hri.ghost_node:main',
            'pantalla_fantasma = muralistas_hri.ghost_ui_node:main',
            'calibracion = muralistas_hri.calibration_node:main',
        ],
    },
)
