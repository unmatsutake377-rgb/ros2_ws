from setuptools import setup
import os
from glob import glob

package_name = 'perception_fusion'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        # 대회 launch 가 parameters=[_pkg_yaml('perception_fusion')] 로 넘긴다 — 설치 안 되면 launch 가 죽는다.
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ssf',
    maintainer_email='ssf@todo.todo',
    description='LiDAR + D455 depth + OAK detector bearings to an object list (/fused_objects)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_fusion = perception_fusion.perception_fusion:main',
        ],
    },
)
