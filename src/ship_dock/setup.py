from setuptools import setup
import os
from glob import glob

package_name = 'ship_dock'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ⬇⬇ 이 줄 추가 (launch 파일 설치)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='유민',
    maintainer_email='you@example.com',
    description='ShipDock node for ship navigation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ship_dock = ship_dock.ship_dock:main',
        ],
    },
)
