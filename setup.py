from setuptools import setup
import os
from glob import glob

package_name = 'd2dtracker_drone_detector'


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('share/' + package_name, ['config.yaml']),
        (os.path.join('share', package_name), glob('launch/*.launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('launch/*.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asmbatati',
    maintainer_email='asmalbatati@hotmail.com',
    description='Drone-to-drone perception pipeline',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = d2dtracker_drone_detector.service_member_function:main',
            'client = d2dtracker_drone_detector.client_member_function:main',
            'img_publisher = d2dtracker_drone_detector.basic_image_publisher:main',
            'img_subscriber = d2dtracker_drone_detector.basic_image_subscriber:main',
            'detection = d2dtracker_drone_detector.detection:main',
            'detect = d2dtracker_drone_detector.detection_node:main',
        ],
    },
)
