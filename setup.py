#!/usr/bin/env python3

from setuptools import setup, find_packages

import os


setup( 
        name='sentinet',
        version='1.0.0',
        description="A Python Sentinet Derivative",
        author="Trace Valade",
        author_email="trace.valade@colorado.edu",
        license='MIT',
        packages=find_packages(),
        install_requires=[
            'numpy',
            'pyzmq',
            'matplotlib'],

        classifiers=[
            'Development Status :: 1 - Planning',
            'Intended Audience :: Science/Research',
            'License :: MIT License',
            'Operating System :: POSIX :: Linux',
            'Programming Language :: Python :: 3',
            'Programming Language :: Python :: 3.4',
            'Programming Language :: Python :: 3.5',
        ],
        url="https://curmc.github.io/",
)
