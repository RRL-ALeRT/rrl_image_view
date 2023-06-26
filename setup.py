#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rrl_image_view'],
    package_dir={'': 'src'},
    scripts=['scripts/rrl_image_view']
)

setup(**d)
