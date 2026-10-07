from setuptools import find_packages, setup

package_name = 'bottle_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/detector.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jemima',
    maintainer_email='jemima.n.t@gmail.com',
    description='Deteccion de botellas con YOLO para Silvanus.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bottle_detector = bottle_perception.bottle_detector:main',
        ],
    },
)
