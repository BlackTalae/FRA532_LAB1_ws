from setuptools import find_packages
from setuptools import setup

setup(
    name='fra532_lab1_package',
    version='0.0.0',
    packages=find_packages(
        include=('fra532_lab1_package', 'fra532_lab1_package.*')),
)
