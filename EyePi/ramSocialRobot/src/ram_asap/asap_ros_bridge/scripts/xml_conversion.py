#!/usr/bin/env python

""" 
This module defines functionality to convert a Ros object
to XML, and XML to a Ros object. It is used by the bridge
to convert data between the Apollo and Ros network.
"""

import genpy
import xml.etree.ElementTree as et
from robotutils.xmlutils import indent, xml2dict, dict2xml
import rospy

def xml_to_rosobject(element, klass):
    """ Convert an xml message to a rosobject of the given class.
    """
    d = xml2dict(element)
    obj = klass()
    _setFields_from_dict(obj, d)
    return obj

def rosobject_to_xml(obj, pretty=False, as_string=True):
    """ Create an xml representation of the given ros object.
    """
    #print(obj)
    value = _rosobject_to_struct(obj)
    return dict2xml(value, 'data', pretty, as_string)

def _rosobject_to_struct(obj):
    if isinstance(obj, genpy.Message):
        # Sub struct
        value = {}
        for f in obj.__slots__:
            value[f] = _rosobject_to_struct(getattr(obj, f))
        return value
    else:
        # Another value (should be compatible with dict2xml)
        return obj

def _setFields_from_dict(obj, d):
    for key, value in d.items():
        field = getattr(obj, key)
        if isinstance(field, genpy.Message):
            # Another dict
            if isinstance(value, dict):
                _setFields_from_dict(field, value)
            else:
                rospy.logerr('Warning: mismatch between struct and Ros object.')
        else:
            # This should just work, also for lists.
            # But we do not test whether the types match
            setattr(obj, key, value)

if __name__ == '__main__':
    import roslib
    roslib.load_manifest('geometry_msgs')
    roslib.load_manifest('turtlesim')
    import geometry_msgs.msg
    Pose = geometry_msgs.msg.Pose
    p = Pose()
    p.orientation.y = 1./7
    
    xml = rosobject_to_xml(p, True)
    print xml    
    obj = xml_to_rosobject(xml, Pose)
    assert obj == p #check that it round-tripped correctly
    print obj
    print xml2dict(xml)
