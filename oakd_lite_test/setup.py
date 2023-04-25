from setuptools import setup
import glob

package_name = 'oakd_lite_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adrian',
    maintainer_email='adrian23fernandez@gmail.com',
    description='Package that test the Turtlebot 4\'s OAKD Lite Camera',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'oakd_pub_test = oakd_lite_test.oakd_pub_test:main',
            'oakd_sub_test = oakd_lite_test.oakd_sub_test:main',
        ],
    },
)
