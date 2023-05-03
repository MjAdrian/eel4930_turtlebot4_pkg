from setuptools import setup

package_name = 'tb4_movement'

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
    maintainer='adrian',
    maintainer_email='adrian23fernandez@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tb4_movement = tb4_movement.tb4_movement:main',
            'static_aiming = tb4_movement.static_aiming:main',
            'static_aiming_y = tb4_movement.static_aiming_y:main',
            'shoot = tb4_movement.shoot_aimed:main'
        ],
    },
)
