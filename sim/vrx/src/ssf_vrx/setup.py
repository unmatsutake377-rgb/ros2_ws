from glob import glob
import os

from setuptools import setup

package_name = 'ssf_vrx'


def _tree(src_dir, dst_prefix):
    """src_dir 아래 파일을 share/<pkg>/<dst_prefix>/... 로 그대로 설치한다 (하위 폴더 포함)."""
    out = []
    for root, _dirs, files in os.walk(src_dir):
        if not files:
            continue
        rel = os.path.relpath(root, src_dir)
        dst = os.path.join('share', package_name, dst_prefix) if rel == '.' \
            else os.path.join('share', package_name, dst_prefix, rel)
        out.append((dst, [os.path.join(root, f) for f in files]))
    return out


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ] + _tree('models', 'models') + _tree('worlds', 'worlds'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SSF',
    maintainer_email='unmatsutake377@gmail.com',
    description='SSF 스택 ↔ VRX(Gazebo Garden) 접착: 배 모델, KABOAT 월드 생성기, 추진기 브릿지',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_bridge = ssf_vrx.sim_bridge:main',
            'make_world = ssf_vrx.make_world:main',
            'make_boat = ssf_vrx.make_boat:main',
        ],
    },
)
