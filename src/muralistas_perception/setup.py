from setuptools import setup

package_name = 'muralistas_perception'

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
    description='Percepción 3D del mural y del pincel.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'generar_arucos = muralistas_perception.generar_arucos:main',
            'perception = muralistas_perception.perception_node:main',
        ],
    },
)
