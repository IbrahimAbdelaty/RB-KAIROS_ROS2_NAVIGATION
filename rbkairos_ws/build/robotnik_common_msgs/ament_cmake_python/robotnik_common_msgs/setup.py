from setuptools import find_packages
from setuptools import setup

setup(
    name='robotnik_common_msgs',
    version='1.0.0',
    packages=find_packages(
        include=('robotnik_common_msgs', 'robotnik_common_msgs.*')),
)
