from setuptools import setup
import os
from glob import glob

package_name = 'ssf_tools'

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
    description='SSF 검증 도구: blackbox(10Hz CSV 로깅) + healthcheck. 둘 다 구독 전용.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'blackbox = ssf_tools.blackbox:main',
            'healthcheck = ssf_tools.healthcheck:main',
        ],
    },
)
