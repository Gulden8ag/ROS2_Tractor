from setuptools import setup
from glob import glob

package_name = 'tractor_analysis' 

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Oliver Ochoa',
    maintainer_email='oliver.ochoa2@iberopuebla.mx',
    description='Skeleton package.',
    license='TODO',
)
