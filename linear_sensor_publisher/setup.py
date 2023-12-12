from setuptools import setup

package_name = 'linear_sensor_publisher'

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
    maintainer='wenpeng',
    maintainer_email='wwang11@wpi.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'linear_publisher = linear_sensor_publisher.publisher_member_function:main',
            'linear_subscriber = linear_sensor_publisher.subscriber_member_function:main',
        ],
    },
)
