#!/usr/bin/env python3

from setuptools import setup, find_packages

setup( 
        name='SentiNet',
        version='1.0',
        description="A Python Version of Sentinet",
        author="Trace Valade",
        author_email="trace.valade@colorado.edu",
        packages=[package for package in find_packages() if package.startswith('sentinet')],
        install_requires=[
            'numpy',
            'pyzmq' ]
       )
