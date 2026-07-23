from setuptools import setup
import os
from glob import glob

package_name = 'ssf_heading'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SSF',
    maintainer_email='you@example.com',
    description='SSF 헤딩 합성: yaw_mux 가 /imu/yaw 의 단독 발행자.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yaw_mux = ssf_heading.yaw_mux:main',
        ],
    },
)
