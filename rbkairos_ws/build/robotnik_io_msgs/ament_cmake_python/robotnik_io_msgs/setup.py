from setuptools import find_packages
from setuptools import setup

setup(
    name='robotnik_io_msgs',
    version='1.0.0',
    packages=find_packages(
        include=('robotnik_io_msgs', 'robotnik_io_msgs.*')),
)
