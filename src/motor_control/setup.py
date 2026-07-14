from setuptools import find_packages, setup

package_name = 'motor_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/motor_control_launch.py']),  # 추가된 부분
        ('share/' + package_name + '/config', ['config/motor_control.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ssf',
    maintainer_email='your_email@example.com',
    description='Package for motor control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_control = motor_control.motor_control:main'
        ],
    },
)
