
from setuptools import setup, find_packages

setup( 
        name='SentiNet',
        version='1.0',
        packages=[package for package in find_packages() if package.startswith('sentinet')],
       )
