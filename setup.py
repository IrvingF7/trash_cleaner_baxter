#!/usr/bin/env python

from distutils.core import setup

setup(name='tcb',
      version='0.0.0',
      description='EE 106A Final Project - Trash Cleanup Baxter',
      package_dir = {'': 'src'},
      packages=['tcb', 'tcb.manipulation', 'tcb.obj_detection', 'tcb.perception'],
      install_requires={'scipy', 'numpy', 'IPython', 'matplotlib', 'opencv-python'}
     )