from setuptools import find_packages, setup

package_name = 'color_shape_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ssfa',
    maintainer_email='minwoochang03@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'img_publisher = color_shape_detector.basic_image_publisher:main',
            'img_subscriber = color_shape_detector.basic_image_subscriber:main', 
            ],
        },
)
