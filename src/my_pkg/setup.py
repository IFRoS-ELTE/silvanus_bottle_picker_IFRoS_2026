from setuptools import find_packages, setup

package_name = 'my_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/demo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.invalid',
    description='Silvanus student template: read the lidar, and drive the base safely.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'scan_watch = my_pkg.scan_watch:main',
            'safe_drive = my_pkg.safe_drive:main',
        ],
    },
)
