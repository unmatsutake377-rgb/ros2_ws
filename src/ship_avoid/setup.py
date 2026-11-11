from setuptools import setup
import os
from glob import glob

package_name = 'ship_avoid'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SSF',
    maintainer_email='you@example.com',
    description='mode 5 장애물 회피 — 낮은 장애물 카메라 지도 + 도리도리 (계약서_도리도리회피_20261112, 미승인)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ship_avoid = ship_avoid.ship_avoid:main',
        ],
    },
)
