import os
import glob
from setuptools import setup

package_name = 'ntrip_client'

setup(
    name=package_name,
    version='1.3.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        # 🚨 launch 파일은 share/<pkg>/launch/ 에 넣는다 (ROS2 관례이자 저장소 나머지 전 패키지의
        #    방식). 예전엔 share/<pkg>/ 에 평평하게 깔려서 launch_files.launch.py 의
        #      os.path.join(dir_ntrip, 'launch', 'ntrip_client_launch.py')
        #    가 파일을 못 찾았고, **대회 실행 launch 가 통째로 죽었다**(2026-08-06 실기 발견).
        #    IncludeLaunchDescription 은 파일이 없으면 예외를 던지고, 그 예외가 launch 전체를
        #    내린다 — 이미 뜬 노드 15개까지 같이 SIGINT 로 종료된다.
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.py')),
        # 계정 **템플릿**만 설치한다. 실제 값은 ~/.ssf/ntrip_credentials.yaml (저장소 밖).
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Rob Fisher',
    author_email='rob.fisher@parker.com',
    maintainer='Rob Fisher',
    maintainer_email='rob.fisher@parker.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='NTRIP client that will publish RTCM corrections to a ROS topic, and optionally subscribe to NMEA messages to send to an NTRIP server',
    license='MIT License',
    tests_require=['pytest'],
    scripts=[
      'scripts/ntrip_ros.py'
    ]
)