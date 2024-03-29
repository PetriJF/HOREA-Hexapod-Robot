from setuptools import setup

package_name = 'hexapod_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='jamespetri28@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "teleop_keyboard_node = hexapod_controller.keyboard_teleop_controller:main",
            "teleop_xbox_gamepad_node = hexapod_controller.xbox_gamepad_teleop_controller:main",
            "teleop_std_gamepad_node = hexapod_controller.gamepad_teleop_controller:main"
        ],
    },
)
