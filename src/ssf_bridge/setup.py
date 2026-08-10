from setuptools import setup
import os
from glob import glob

package_name = 'ssf_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 🚨 glob('launch/*') 로 두면 __pycache__ 까지 잡혀 빌드가 깨진다
        #    (ntrip_client 에서 실제로 겪음). 확장자를 명시한다.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SSF',
    maintainer_email='you@example.com',
    description='Motor_run ↔ Arduino Mega 시리얼 브릿지 (명령 송신 + 상태 수신).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge = ssf_bridge.bridge:main',
        ],
    },
)
