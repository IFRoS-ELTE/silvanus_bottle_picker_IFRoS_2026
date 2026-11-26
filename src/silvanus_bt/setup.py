from glob import glob

from setuptools import find_packages, setup

package_name = 'silvanus_bt'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jemima',
    maintainer_email='jemima.n.t@gmail.com',
    description='Silvanus behaviour tree (py_trees_ros): explore and collect bottles.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'silvanus_bt = silvanus_bt.tree:main',
        ],
    },
)
