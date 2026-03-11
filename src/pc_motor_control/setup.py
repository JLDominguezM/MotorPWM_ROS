from setuptools import find_packages, setup

package_name = 'pc_motor_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dominguez',
    maintainer_email='A01285873@tec.mx',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor_controller = pc_motor_control.motor_controller:main',
            'input_node = pc_motor_control.input_node:main',
            'control_node = pc_motor_control.control_node:main',
            'tuning_node = pc_motor_control.tuning_node:main',
            'odometry_node = pc_motor_control.odometry_node:main',
            
        ],
    },
)
