""" robottools.xml
Tools for xml processing.

"""

import sys
import os
import xml.etree.ElementTree as et

if sys.version_info >= (3,):
    basestring = str

# Try importing GenP
try:
    import genpy
    Time = genpy.Time
except Exception:
    genpy = None
    class Time:
        def __init__(self, secs, nsecs):
            self.secs, self.nsecs = secs, nsecs
        def __repr__(self):
            return "<Time %i secs, %i nsecs>" % (self.secs, self.nsecs)



def indent(elem, level=0):
    """ For pretty printing.
    Taken from http://effbot.org/zone/element-lib.htm#prettyprint
    """
    i = "\n" + level*"  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for elem in elem:
            indent(elem, level+1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i



def xml2dict(xml):
    """ xml2dict(xml)
    Create a Python dict from xml. The input can be a string or an
    ElementTree.Element object.
    """
    
    # Get root element
    if et.iselement(xml):
        root = xml
    elif isinstance(xml, basestring):
        root = et.fromstring(xml)
    else:
        raise ValueError('xml2dict accepts a string or ElementTree.Element.')
    
    # Iterate over children to compose dict
    d = {}
    for child in root:
        if not child.tag:
            continue
        d[child.tag] = _xml2dict(child)
    
    # Return dict
    return d


def _xml2dict(element, path=''):
    """ Do actual parsing from xml to dict.
    """
    
    # Init
    path = path + '/'*bool(path) + element.tag
    type = element.get('type', '').lower()
    nchilden = len(element)
    value = None
    
    # Construct the Python value
    try:
        if not type:
            print("Warning: expected attribute 'type' on xml path '%s'." % path)
        elif type == 'float':
            value = float(element.text)
        elif type == 'int':
            value = int(element.text)
        elif type == 'bool':
            value = element.text=='True'
        elif type == 'str':
            value = element.text or ''  # Empty text resolves to None
        elif type == 'tuple':
            value = [_xml2dict(child, path) for child in element]
            value = tuple(value)
        elif type == 'dict':
            value = {}
            for child in element:
                value[child.tag] = _xml2dict(child, path)
        
        elif type == 'time':
            tmp = [int(i) for i in element.text.split(' ')]
            value = Time(*tmp)
        elif type == 'null':
            value = None
        
        else:
            print("Warning: unknown type '%s' for xml path '%s'." % (type, path))
    except Exception as err:
        print("Warning: could not parse xml path '%s': %s" % (path, str(err)))
    
    # In case we could not parse it, at least give the contents
    if value is None:
        value = element.text
    
    return value



def dict2xml(d, tagname='data', pretty=False, as_string=True):
    """ dict2xml(d, tagname='data', pretty=False, as_string=True)
    Create an xml string from a Python dict. Will add indentation if
    pretty is True. Will return a Python ElementTree object if as_string
    is False.
    """
    # Create root element
    root = et.Element(tagname)
    
    # Parse items
    for key, val in d.items():
        _dict2xml(root, key, val)
    
    # Return as string
    if pretty:
        indent(root)
    
    if as_string:
        if sys.version_info >= (3,):
            return et.tostring(root, 'unicode')
        else:
            return et.tostring(root, 'utf-8')
    else:
        return root


def _dict2xml(parent, key, value):
    """Do actual parsing from dict to xml.
    """
    # Create sub element
    child = et.SubElement(parent, key)
    
    # Parse value
    if isinstance(value, float):
        child.set('type', 'float')
        child.text = repr(value)
    elif isinstance(value, bool):
        child.set('type', 'bool')
        child.text = str(value)
    elif isinstance(value, int):
        child.set('type', 'int')
        child.text = repr(value)
    elif isinstance(value, basestring):
        child.set('type', 'str')
        child.text = value
    elif isinstance(value, (list, tuple)):
        child.set('type', 'tuple')
        for subval in value:
            _dict2xml(child, 'e', subval)
    elif isinstance(value, dict):
        child.set('type', 'dict')
        for key, subval in value.items():
            _dict2xml(child, key, subval)
    
    elif isinstance(value, Time):
        child.set('type', 'time')
        child.text = '%i %i' % (value.secs, value.nsecs)
    elif value is None:
        child.set('type', 'null')  # Ros seems to pass None for empty strings
        child.text = ''
    
    else:
        typename = value.__class__.__name__
        print('warning: dict2xml does not support objects of type %s.' % typename)
        child.set('type', typename)
        text = repr(value)
        if len(text) > 64:
            text = text[:61] + '...'
        child.text = text


if __name__ == '__main__':
    
    d1 = {'foo': 3.0, 'bar': 4, 'spam':'asdasd'}
    d2 = {'L': (1,2,3.0, 'lol', d1, (1,2,3.0)),
          'D': d1}
    
    xml = dict2xml(d2, pretty=True)
    print(xml)
    d3 = xml2dict(xml)
    
    assert d2 == d3
    
    if False:
        print(dict2xml({'foo': sys}))
        import numpy as np; print(dict2xml({'foo': np.zeros((50,50))}))