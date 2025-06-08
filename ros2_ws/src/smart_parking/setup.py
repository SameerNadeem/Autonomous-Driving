import warnings
from setuptools.command.easy_install import EasyInstallDeprecationWarning
warnings.filterwarnings("ignore", category=EasyInstallDeprecationWarning)

from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'smart_parking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*_launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'parking_node = smart_parking.parking_node:main'
        ],
    },
)
