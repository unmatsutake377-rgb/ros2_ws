from setuptools import setup

package_name = 'ship_goal_angle'  # your package name

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ship_goal_angle_launch_file.launch.py']),
        ('share/' + package_name + '/config', ['config/ship_goal_angle.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Package description',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ship_goal_angle_node = ship_goal_angle.ship_goal_angle:main',  # ship_goal_angle_node entry point
        ],
    },
)
