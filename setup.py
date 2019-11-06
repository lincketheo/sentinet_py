#!/usr/bin/env python3

from setuptools import setup, find_packages

setup( 
        name='SentiNet',
        version='1.0.0',
        description="A Python Sentinet Derivative",
        author="Trace Valade",
        author_email="trace.valade@colorado.edu",
        license='MIT',
        packages=[package for package in find_packages() if package.startswith('sentinet')],
        install_requires=[
            'numpy',
            'pyzmq',
            'matplotlib'],

        classifiers=[
            'Development Status :: 1 - Planning',
            'Intended Audience :: Science/Research',
            'License :: OSI Approved :: BSD License',
            'Operating System :: POSIX :: Linux',
            'Programming Language :: Python :: 2',
            'Programming Language :: Python :: 2.7',
            'Programming Language :: Python :: 3',
            'Programming Language :: Python :: 3.4',
            'Programming Language :: Python :: 3.5',
        ],
        url="https://curmc.github.io/",
)
