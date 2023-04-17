from setuptools import setup

package_name = 'autoro_navigation'

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
    maintainer='mshen',
    maintainer_email='mshen@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test = autoro_navigation.test_node:main',
            'get_costmap_test = autoro_navigation.get_costmap_test:main',
            'get_map_test = autoro_navigation.get_map_test:main',
            'talker = autoro_navigation.publisher_member_function:main',
        ],
    },
)
