# Copyright (C) 2022 twyleg
import os
from setuptools import find_packages, setup


def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()


setup(
    name="track_generator",
    version=read('VERSION.txt'),
    author="Torsten Wylegala & Adam Karsten",
    author_email="mail@twyleg.de & a.karsten@ostfalia.de",
    description=("Track generator"),
    license="GPL 3.0",
    keywords="svg model vehicles track",
    url="https://github.com/karad94/track_generator",
    packages=find_packages(),
    include_package_data=True,
    long_description=read('README.md'),
    install_requires=[
        'pytransform3d',
        'numpy',
        'drawSvg',
        'twine',
        'PyQt5'
    ],
    entry_points={
        'console_scripts': [
            'track_generator = track_generator.starter:start',
        ]
    }
)
