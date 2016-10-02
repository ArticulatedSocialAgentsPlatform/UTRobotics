"""
Utility package for functionality that is common to different parts
of the robot platform. 

The package must be installed before it can be used:::
    
    sudo setup.py install

"""

__version__ = '0.1.dev'

from .bcfinterface import BCFInterface
from .config import set_config, get_config
from .xmlutils import xml2dict, dict2xml
