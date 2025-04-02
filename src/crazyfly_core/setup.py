from setuptools import find_packages, setup

package_name = 'crazyfly_core'

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
    maintainer='rsl',
    maintainer_email='Ryanshiau1@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cf_driver = crazyfly_core.cf_driver:main',
            'fake_thrust_pub = crazyfly_core.fake_thrust_pub:main',
            'optitrack_subscriber = crazyfly_core.optitrack_subscriber:main',
            'cf_thrust_controller = crazyfly_core.cf_thrust_controller:main',
            'cf_command_controller = crazyfly_core.cf_command_controller:main',
            'optitrack_subscriber2 = crazyfly_core.optitrack_subscriber2:main',
            'cf_command_controller2 = crazyfly_core.cf_command_controller2:main',
        ],
    },
)
#In case this doesnt work, delete the last two lines in the console scripts 
#then it will be in the origional working format again 
#this includes: optitrack_2 and cf_command_2 
