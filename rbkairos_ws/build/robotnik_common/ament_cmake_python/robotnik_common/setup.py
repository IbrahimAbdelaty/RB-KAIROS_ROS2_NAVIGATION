from setuptools import find_packages
from setuptools import setup

setup(
    name='robotnik_common',
    version='1.1.0',
    packages=find_packages(
        include=('robotnik_common', 'robotnik_common.*')),
)
