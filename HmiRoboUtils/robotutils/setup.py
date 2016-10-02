# -*- coding: utf-8 -*-
# Copyright (C) 2012, Science Applied v.o.f.

""" 
Setup script for robot platform.
"""

import os
import sys
from distutils.core import setup

name = 'robotutils'
description = 'Utilities for the Robot Platform.'


# Get version and docstring
__version__ = None
__doc__ = ''
docStatus = 0 # Not started, in progress, done
initFile = os.path.join(os.path.dirname(__file__), '__init__.py')
for line in open(initFile).readlines():
    if (line.startswith('__version__')):
        exec(line.strip())
    elif line.startswith('"""'):
        if docStatus == 0:
            docStatus = 1
            line = line.lstrip('"')
        elif docStatus == 1:
            docStatus = 2
    if docStatus == 1:
        __doc__ += line


setup(
    name = name,
    version = __version__,
    author = 'Science Applied v.o.f.',
    license = '(new) BSD?',
        
    description = description,
    long_description = __doc__,
    
    platforms = 'any',
    provides = ['robotutils'],
    
    packages = ['robotutils', ],
    py_modules = ['robotutils.config', 'robotutils.xmlutils'],
    package_dir = {'robotutils': '.'},  # must be a dot, not an empty string
    )
