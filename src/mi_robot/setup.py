from setuptools import setup
import os
from glob import glob

package_name = 'mi_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name] if os.path.isdir(package_name) else [],
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'),   glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'rviz'),   glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Oliver Ochoa',
    maintainer_email='oliver.ochoa2@iberopuebla.mx',
    description='Single-package robot with URDF + launch + RViz',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
