#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['platforms_server',
              'path_planner',
              'vision',
              'visualize_utils',
              'mqtt_utils',
              'vrep_communicator'],
    package_dir={'': 'src'},
)

setup(**d)
