#!/usr/bin/env python

from setuptools import setup

setup(name='tcb',
      version='1.0',
      description='EE 106A Final Project - Trash Cleanup Baxter',
      package_dir = {'': 'src'},
      packages=['tcb', 'tcb.manipulation', 'tcb.obj_detection', 'tcb.perception'],
     )