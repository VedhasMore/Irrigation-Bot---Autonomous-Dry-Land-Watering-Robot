from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'irrigation_bot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='your_email@example.com',
    author='ros',
    author_email='your_email@example.com',
    description='Irrigation robot for detecting and watering dry areas',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'irrigation_bot = irrigation_bot.irrigation_bot:main',
        ],
    },
)

