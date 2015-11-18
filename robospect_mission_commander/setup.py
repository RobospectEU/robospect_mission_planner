#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['robospect_mission_commander'],
    package_dir={'': 'src'},
    scripts=['scripts/rqt_robospect_mission_commander']
)

setup(**d)
