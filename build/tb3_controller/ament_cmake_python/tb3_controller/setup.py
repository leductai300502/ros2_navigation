from setuptools import find_packages
from setuptools import setup

setup(
    name='tb3_controller',
    version='0.0.0',
    packages=find_packages(
        include=('tb3_controller', 'tb3_controller.*')),
)
