from setuptools import find_packages
from setuptools import setup

setup(
    name='color_shape_detector',
    version='0.0.0',
    packages=find_packages(
        include=('color_shape_detector', 'color_shape_detector.*')),
)
