from setuptools import find_packages, setup

package_name = 'geomagic_control_py'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'ABBRobotEGM',
    ],
    zip_safe=True,
    maintainer='asantos',
    maintainer_email='alvarosang2003@gmail.com',
    description='Package for controlling the Geomagic Touch haptic device using ROS 2.',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'haptic_to_cartesian = geomagic_control_py.haptic_to_cartesian:main',
            'cartesian_to_haptic = geomagic_control_py.cartesian_to_haptic:main',
        ],
    },
)
