from setuptools import find_packages, setup

package_name = 'precision_landing'

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
    maintainer='huyen',
    maintainer_email='huyen@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'landing_controller_node = precision_landing.landing_controller_node:main',
		'mock_marker_publisher = precision_landing.mock_marker_publisher:main',
        'aruco_detector_node = precision_landing.aruco_detector_node:main',
	'camera_reader_node = precision_landing.camera_reader_node:main',
	'gps_vio_switching_node = precision_landing.gps_vio_switching_node:main',
	'lora_protocol_node = precision_landing.lora_protocol_node:main',
        'lora_ground_station_mock = precision_landing.lora_ground_station_mock:main',
	],
    },
)
