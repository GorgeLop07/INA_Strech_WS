from setuptools import setup

package_name = 'muralistas_sim'

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
    description='Simulación MuJoCo del Stretch 3 pintando un mural.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mural_sim = muralistas_sim.sim_node:main',
        ],
    },
)
