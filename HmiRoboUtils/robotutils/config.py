#!/usr/bin/env python3
# Copyright (C) 2013, Science Applied v.o.f.

"""
Module for reading and setting configuration files. Provides
get_config and set_config functions.
"""

import sys
import os
import xml.etree.ElementTree as et

from robotutils.xmlutils import dict2xml, xml2dict

if sys.version_info >= (3,):
    basestring = str


def get_config(componentname, **default):
    """ get_config(componentname, **default)
    
    Get configuration values for this computer.
    
    Componentname is for instance 'bridge' or 'apollo'. Returns
    a dict equal to the given kwargs, but updated with the values read
    from the configuration file.
    """
    if not componentname.replace('_', '').isalnum():
        raise ValueError('Invalid application name.')
    # Get filename for config
    fname = os.path.join(appdata_dir('robotplatform'), componentname+'.xml')
    # Init dict
    d = default.copy()    
    # Load dict?
    if os.path.isfile(fname):
        #s = open(fname, 'rt', encoding='utf-8').read()
        s = open(fname, 'rb').read().decode('utf-8') # py 2.x compatible
        d.update(_config_xml_to_dict(s))
    # Return
    return d


def set_config(componentname, **values):
    """ set_config(componentname, **values)
    
    Set configuration values for this computer.
    
    Where componentname is for instance 'bridge', 'apollo'. Updates
    the given values in the configuration file. Other values in the
    config file are left as is. Returns a dict with the full
    configuration.
    
    Supported datatypes are int, float, str, and tuple. Configuration
    values can be deleted by setting them to None. 
    """
    if not componentname.replace('_', '').isalnum():
        raise ValueError('Invalid application name.')
    # Get filename for config
    fname = os.path.join(appdata_dir('robotplatform'), componentname+'.xml')
    # Init dict
    d = dict()
    # Load dict?
    if os.path.isfile(fname):
        #s = open(fname, 'rt', encoding='utf-8').read()
        s = open(fname, 'rb').read().decode('utf-8') # py 2.x compatible
        d.update(_config_xml_to_dict(s))
    # Update with given
    for key, val in values.items():
        if val is None:
           d.pop(key, None)
        else:
            d[key] = val
    # Write back  
    s = _config_dict_to_xml(d)
    #open(fname, 'wt', encoding='utf-8').write(s)
    open(fname, 'wb').write(s.encode('utf-8')) # utf-8
    # Return
    return d


def _config_xml_to_dict(s):
    # Get root and check it
    root = et.fromstring(s)
    if root.tag.lower() == 'config':
        return {}  # Old version: reset
    elif root.tag.lower() == 'configuration':
        return xml2dict(root)
    else:
        raise ValueError('Invalid xml root tag for config.')


def _config_dict_to_xml(d):
    return dict2xml(d, 'configuration', True)


# From pyzolib/paths.py (https://code.google.com/p/pyzolib/source/browse/paths.py)
import os, sys
def appdata_dir(appname=None, roaming=False, macAsLinux=False):
    """ appdata_dir(appname=None, roaming=False,  macAsLinux=False)
    Get the path to the application directory, where applications are allowed
    to write user specific files (e.g. configurations). For non-user specific
    data, consider using common_appdata_dir().
    If appname is given, a subdir is appended (and created if necessary). 
    If roaming is True, will prefer a roaming directory (Windows Vista/7).
    If macAsLinux is True, will return the Linux-like location on Mac.
    """
    
    # Define default user directory
    userDir = os.path.expanduser('~')
    
    # Get system app data dir
    path = None
    if sys.platform.startswith('win'):
        path1, path2 = os.getenv('LOCALAPPDATA'), os.getenv('APPDATA')
        path = (path2 or path1) if roaming else (path1 or path2)
    elif sys.platform.startswith('darwin') and not macAsLinux:
        path = os.path.join(userDir, 'Library', 'Application Support')
    # On Linux and as fallback
    if not (path and os.path.isdir(path)):
        path = userDir
    
    # Maybe we should store things local to the executable (in case of a 
    # portable distro or a frozen application that wants to be portable)
    prefix = sys.prefix
    if getattr(sys, 'frozen', None): # See application_dir() function
        prefix = os.path.abspath(os.path.dirname(sys.path[0]))
    for reldir in ('settings', '../settings'):
        localpath = os.path.abspath(os.path.join(prefix, reldir))
        if os.path.isdir(localpath):
            try:
                open(os.path.join(localpath, 'test.write'), 'wb').close()
                os.remove(os.path.join(localpath, 'test.write'))
            except IOError:
                pass # We cannot write in this directory
            else:
                path = localpath
                break
    
    # Get path specific for this app
    if appname:
        if path == userDir:
            appname = '.' + appname.lstrip('.') # Make it a hidden directory
        path = os.path.join(path, appname)
        if not os.path.isdir(path):
            os.mkdir(path)
    
    # Done
    return path


if __name__ == '__main__':
    set_config('test', foo='lala', bar=(1,2,3.0))
    print(get_config('test'))
    