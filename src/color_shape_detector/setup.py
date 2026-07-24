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
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ssf',
    maintainer_email='dkswnsdud10@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "basic_image_publisher = color_shape_detector.basic_image_publisher:main",
            "basic_image_subscriberdock = color_shape_detector.basic_image_subscriberdock:main",
            "basic_image_subscriberhsv = color_shape_detector.basic_image_subscriberhsv:main",
            "basic_image_subscribergate = color_shape_detector.basic_image_subscribergate:main",
            "basic_image_subscriberturn = color_shape_detector.basic_image_subscriberturn:main",
        ],
    },
)
