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
        (os.path.join('share', package_name), glob('config/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohamed Abdelkader',
    maintainer_email='mohamedashraf123@gmail.com',
    description='Drone-to-drone perception pipeline',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_node = d2dtracker_drone_detector.detection_node:main',
        ],
    },
)
