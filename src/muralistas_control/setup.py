from setuptools import setup

package_name = 'muralistas_control'

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
    description='Control: trazos del mural -> base, elevador y brazo del Stretch (FollowJointTrajectory), apoyar/levantar el pincel y paro.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'control = muralistas_control.control_node:main',
            'prueba_articulaciones = muralistas_control.prueba_articulaciones:main',
            'teleop_base = muralistas_control.teleop_base:main',
        ],
    },
)
