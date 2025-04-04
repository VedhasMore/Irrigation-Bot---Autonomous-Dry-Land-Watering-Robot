from setuptools import setup

package_name = 'irrigation_bot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Simple controller for irrigation bot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'simple_mover = irrigation_bot_controller.simple_mover:main'
        ],
    },
)
