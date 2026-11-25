from glob import glob

from setuptools import setup

package_name = 'uwheel_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubicomp',
    maintainer_email='gyehd0107@gmail.com',
    description='U-Wheel joystick collision avoidance + 2D mapping',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'scan_merger = uwheel_nav.scan_merger:main',
            'fastlio_odom_bridge = uwheel_nav.fastlio_odom_bridge:main',
            'teleop_avoidance = uwheel_nav.teleop_avoidance:main',
            'twist_sign_fix = uwheel_nav.twist_sign_fix:main',
        ],
    },
)
