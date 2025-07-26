from setuptools import find_packages
from setuptools import setup

setup(
    name='robotnik_navigation_msgs',
    version='1.0.0',
    packages=find_packages(
        include=('robotnik_navigation_msgs', 'robotnik_navigation_msgs.*')),
)
