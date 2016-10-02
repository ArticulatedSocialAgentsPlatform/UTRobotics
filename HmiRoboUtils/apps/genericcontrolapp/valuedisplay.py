#!/usr/bin/env python3
# Copyright (C) 2013, Science Applied v.o.f.

""" Qt Widgets for displaying values.
"""

import sys
import os
import time
import math

import stomp

from robotutils.qt import QtGui, QtCore
from robotutils import xml2dict, dict2xml, get_config, set_config
from robotutils.xmlutils import Time
from robotutils import BCFInterface

import messagedisplay

LABEL_COLOR = '#ccf'
VALUE_COLOR = '#eee'


if sys.version_info >= (3,):
    basestring = str


class BCFProxy(QtCore.QObject):
    
    faderValueChanged = QtCore.Signal(int, int)
    encoderValueChanged = QtCore.Signal(int, int)
    
    def __init__(self):
        QtCore.QObject.__init__(self)
        port = get_config('bcf', port=0)['port']
        try:
            self._interface = BCFInterface(port, port)
        except Exception:
            print('could not connect to BCF2000')
            return
        else:
            #self._interface.dance(1)
            self._interface.fader_callback = self.onFader
            self._interface.encoder_callback = self.onEncoder
        
    def onFader(self, id, value):
        self.faderValueChanged.emit(id, value)
    
    def onEncoder(self, id, value):
        self.encoderValueChanged.emit(id, value)
    
    def set_fader(self, id, val):
        self._interface.set_fader(id, val)
    
    def set_encoder(self, id, val):
        self._interface.set_encoder(id, val)



# Instantiate singleton BCFInterface object
BCF = BCFProxy()



class BaseValueDisplay(QtGui.QFrame):
    """ Base for GUI elements to display values (of messages).
    """
    def __init__(self, parent, editable):
        QtGui.QFrame.__init__(self, parent)
        self._editable = editable
        self.setStyleSheet("BaseValueDisplay { border-radius: 5px; background-color:%s; padding: 2px;}" % VALUE_COLOR)
    
    def setMainWidget(self, widget):
        layout = QtGui.QHBoxLayout(self)
        layout.setContentsMargins(0,0,0,0)
        self.setLayout(layout)
        layout.addWidget(widget)
    
    def pushValue(self, value):
        pass
    
    def pullValue(self):
        return None
    
    def onValueChanged(self):
        """ Should be called when a value changes.
        """
        # Get Display object
        parent = self.parent()
        while not isinstance(parent, (BaseValueDisplay, messagedisplay.Display)):
            parent = parent.parent()
        # Send message
        parent.onValueChanged()



class StructDisplay(BaseValueDisplay):
    """ Display a struct using widgets and controls.
    There can be StructDisplays inside a StructDisplay.
    """
    
    def __init__(self, parent, editable, struct):
        BaseValueDisplay.__init__(self, parent, editable)
        
        self.setStyleSheet("StructDisplay { border: 1px solid %s; border-radius: 5px;}" % VALUE_COLOR)
        
        # Create form layout
        layout = QtGui.QFormLayout(self)
        self.setLayout(layout)
        
        self._structure = {}
        
        for key in sorted(struct.keys()):
            item = struct[key]
            
            # Create label
            label = QtGui.QLabel(key, self)
            label.setStyleSheet("QLabel { border-radius: 5px; background-color:%s; padding: 4px;}" % LABEL_COLOR)
            
            # Create field, depending on type
            if isinstance(item, dict):
                field = StructDisplay(self, editable, item)  # Recursion
            elif isinstance(item, (tuple, list)):
                field = TupleDisplay(self, editable, item) # Recursion
            elif isinstance(item, float):
                field = FloatDisplay(self, editable)
            elif isinstance(item, bool):
                field = BoolDisplay(self, editable)
            elif isinstance(item, int):
                field = IntDisplay(self, editable)
            elif isinstance(item, basestring):
                field = StrDisplay(self, editable)
            elif isinstance(item, Time):
                field = TimeDisplay(self, editable)
            else:
                # Don't know this type (yet)
                field = QtGui.QLabel(str(item)+': unknown type', self)
            
            # Add row
            layout.addRow(label, field)
            # Keep structure
            self._structure[key] = field
            field._dummyValue = item  # For items that are not dynamic
    
    def pushValue(self, struct):
        for key in struct:
            # Get object
            try:
                object = self._structure[key]
            except Exception:
                print('Cannot push key %s' % key)
            else:
                if hasattr(object, 'pushValue'):
                    object.pushValue( struct[key] )
    
    
    def pullValue(self):
        d = {}
        for key in self._structure:
            # Get object
            object = self._structure[key]
            if hasattr(object, 'pullValue'):
                d[key] =  object.pullValue()
            elif hasattr(object, '_dummyValue'):
                d[key] =  object._dummyValue
            else:
                print('Cannot pull key %s' % key)
        return d
    


class TupleDisplay(BaseValueDisplay): 
    
    _typemap = {'float': 0.0, 'bool': False, 'int': 0, 'str': '', 'Time': Time(0,0)}
    
    def __init__(self, parent, editable, elements):
        BaseValueDisplay.__init__(self, parent, editable)
        
        self.setStyleSheet("TupleDisplay { border: 1px solid %s; border-radius: 5px; }" % VALUE_COLOR)
        
        # Initialize item widgets
        self._items = []
        
        # Create button to add new elements to the tuple
        self._addBut = QtGui.QToolButton(self)
        self._addBut.setText('+')
        self._addBut.setAutoRaise(True)
        self._addBut.setPopupMode(self._addBut.InstantPopup)
        #
        self._addMenu = QtGui.QMenu()
        self._addBut.setMenu(self._addMenu)
        self._addMenu.triggered.connect(self.onAdd)
        for name in self._typemap.keys():
            self._addMenu.addAction(name)
        self._addMenu.addSeparator()
        for name in ['remove first', 'remove last']:
            self._addMenu.addAction(name)
        
        # Initialize layout
        self._layout = QtGui.QHBoxLayout(self)
        self._layout.setContentsMargins(2,2,2,2)
        self.setLayout(self._layout)
        #
        self._layout.addWidget(self._addBut, 0)
        self._layout.addStretch(1)
        
        # Init "example" elememts
        for value in elements:
            self.addElement(value)
    
    
    def onAdd(self, action):
        typename = str(action.text())
        # Remove an item?
        if 'remove' in typename:
            if self._items:
                item = self._items[0] if 'first' in typename else self._items[-1]
                self._items.remove(item)
                self._layout.removeWidget(item)
                item.close()
            return
        # Create field, depending on type
        value = self._typemap.get(typename, None)
        if value is None:
            raise Exception('Unkown type name for tuple element.')
        else:
            self.addElement(value)
    
    
    def addElement(self, item):
        editable = self._editable
        # Create field, depending on type
        
        if isinstance(item, float):
            field = FloatDisplay(self, editable, False)
        elif isinstance(item, bool):
            field = BoolDisplay(self, editable)
        elif isinstance(item, int):
            field = CompactIntDisplay(self, editable)
        elif isinstance(item, basestring):
            field = StrDisplay(self, editable)
        elif isinstance(item, Time):
            field = TimeDisplay(self, editable)
        else:
            # Don't know this type (yet)
            field = QtGui.QLabel(str(item) + ': unkown type', self)
        # Add 
        self._layout.insertWidget(len(self._items), field, 0)
        self._items.append(field)
    
    
    def pushValue(self, value):
        # Dynamically add elements if necessary
        # Note that in theory, the type of an element could change,
        # which would produce an error if we for instance try to
        # pushValue a string to a FloatDisplay. However, this probably
        # won't happen in practice (I think).
        i = 0
        for i, val in enumerate(value):
            if i >= len(self._items):
                self.addElement(val)
            self._items[i].pushValue(val)
        # And remove too
        for item in self._items[i+1:]:
            self._items.remove(item)
            self._layout.removeWidget(item)
            item.close()
    
    def pullValue(self):
        return tuple([e.pullValue() for e in self._items])



class CompactFloatDisplay(BaseValueDisplay):
    def __init__(self, parent, editable):
        BaseValueDisplay.__init__(self, parent, editable)
        # Create spinbox
        self._w = QtGui.QDoubleSpinBox(self)
        self.setMainWidget(self._w)
        #
        self._w.setRange(-99, 99)
        self._w.valueChanged.connect(self.onValueChanged)
        self._w.setReadOnly(not editable)
    
    def pushValue(self, value):
        self._w.setValue(value)
    
    def pullValue(self):
        return self._w.value()


class CompactBoolDisplay(BaseValueDisplay):
    def __init__(self, parent, editable):
        BaseValueDisplay.__init__(self, parent, editable)
        # Create checkbox
        self._w = QtGui.QCheckBox(self)
        self.setMainWidget(self._w)
        #
        self._w.stateChanged.connect(self.onValueChanged)
    
    def pushValue(self, state):
        self._w.setCheckState(state)
    
    def pullValue(self):
        return self._w.checkState()==2



class CompactIntDisplay(BaseValueDisplay):
    def __init__(self, parent, editable):
        BaseValueDisplay.__init__(self, parent, editable)
        # Create spinbox
        self._w = QtGui.QSpinBox(self)
        self.setMainWidget(self._w)
        #
        self._w.setRange(-99, 99)
        self._w.valueChanged.connect(self.onValueChanged)
        self._w.setReadOnly(not editable)
    
    def pushValue(self, value):
        self._w.setValue(value)
    
    def pullValue(self):
        return self._w.value()


class ScalarDisplay(BaseValueDisplay):
    """ Scalars can be displayed in different ways: using 
    a slider, a spinbox, or act as a proxy to a motor fader.
    """
    
    def __init__(self, parent, editable, showSlider=True):
        BaseValueDisplay.__init__(self, parent, editable)
        
        # To store value and a flag to keep track when it is changed
        self._value = self.TYPE()
        self._valueChangeFlag = False
        
        # Init id's of faders and encoders that this float value is registered to
        self._bcf_fader = 0
        self._bcf_encoder = 0
        
        # Create button to change range etc
        self._menuBut = QtGui.QToolButton(self)
        self._menuBut.setText('')
        self._menuBut.setAutoRaise(True)
        self._menuBut.setPopupMode(self._menuBut.InstantPopup)
        #
        self._menu = QtGui.QMenu()
        self._menuBut.setMenu(self._menu)
        self._menu.triggered.connect(self.onMenuAction)
        self._menu.addAction('Set range')
        self._menu.addSeparator()
        self._menu.addAction('Show as slider')
        self._menu.addAction('Show as spinbox')
        self._menu.addSeparator()
        self._menu.addAction('Associate BCF-2000 fader')
        self._menu.addAction('Associate BCF-2000 encoder')
        
        # Create spinbox
        if self.TYPE is float:
            self._spinbox = QtGui.QDoubleSpinBox(self)
        else:
            self._spinbox = QtGui.QSpinBox(self)
        self._spinbox.setReadOnly(not editable)
        
        # Create slider
        self._slider = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self._slider.setMinimumWidth(50)
        self._slider.setDisabled(not editable)
        
        # Create label for its value
        self._label = QtGui.QLabel(self)
        self._label.setText('')
        #self._label.setMinimumWidth(50)
        self._label.setMinimumWidth(70)
        
        # Init visualization mode
        if showSlider:
            self._spinbox.hide()
        else:
            self._slider.hide()
            self._label.hide()
        
        # Init range 
        self._range = -5, 5
        self._setRange(*self._range)
        self._changeValue(self._value)
        
        # Connect bcf
        BCF.faderValueChanged.connect(self._onFaderChanged)
        BCF.encoderValueChanged.connect(self._onEncoderChanged)
        
        # Connect widgets
        self._spinbox.valueChanged.connect(self._onSpinBoxChanged)
        self._slider.valueChanged.connect(self._onSliderChanged)
        
        # Layout
        layout = QtGui.QHBoxLayout(self)
        layout.setSpacing(2)
        self.setLayout(layout)
        #
        layout.addWidget(self._menuBut)
        layout.addWidget(self._spinbox)
        layout.addWidget(self._slider)
        layout.addWidget(self._label)
    
    
    def onMenuAction(self, action):
        actionText = str(action.text()).lower()
        
        if 'range' in actionText:
            # Ask for range
            rangeText = '%1.2g...%1.2g' % self._range
            rangeText = QtGui.QInputDialog.getText(self, 'Range', 'Give new range:', text=rangeText)
            if isinstance(rangeText, tuple): rangeText = rangeText[0]
            # Parse
            if rangeText:
                parts = [self.TYPE(v) for v in rangeText.split('..') if v]
                if len(parts) == 2:
                    self._setRange(*parts)
        
        elif 'bcf' in actionText:
            # Use the motor fader!
            nr = QtGui.QInputDialog.getInt(self, 'BCF2000 number', 'Give fader/encoder number:')
            if isinstance(nr, tuple): nr = nr[0]
            if 'fader' in actionText:
                self._bcf_fader = nr
            elif 'encoder' in actionText:
                self._bcf_encoder = nr
        
        elif 'show' in actionText:
            # Change visualization mode
            if 'spinbox' in actionText:
                self._slider.hide()
                self._label.hide()
                self._spinbox.show()
            elif 'slider' in actionText:
                self._spinbox.hide()
                self._slider.show()
                self._label.show()
    
    def _setRange(self, *range):
        range = min(range), max(range)
        self._range = tuple(range)
        self._slider.setRange(  range[0]/self.SLIDER_SCALE, 
                                range[1]/self.SLIDER_SCALE)
        self._spinbox.setRange(*range)
        self._changeValue(self._value)
    
    
    def _onFaderChanged(self, id, value):
        if self._bcf_fader == id:
            value = value / 127.0
            value = value * (self._range[1]-self._range[0]) + self._range[0]
            self._changeValue(value)
    
    def _onEncoderChanged(self, id, value):
        if self._bcf_encoder == id:
            value = value / 127.0
            value = value * (self._range[1]-self._range[0]) + self._range[0]
            self._changeValue(value)
    
    def _onSliderChanged(self):
        value = self._slider.value() * self.SLIDER_SCALE
        self._changeValue(value)
    
    def _onSpinBoxChanged(self):
        value = self._spinbox.value()
        self._changeValue(value)
    
    def _changeValue(self, value):
        """ Update the value at different places, but make sure not
        to end up in an endless loop.
        """
        value = self.TYPE(value)
        if not self._valueChangeFlag:
            self._valueChangeFlag = True
            try:
                # Set value ...
                self._value = value
                self._slider.setValue(value/self.SLIDER_SCALE)
                self._spinbox.setValue(value)
                #self._label.setText('%1.2g' % value)
                self._label.setText('%1.2g [%1.2g...%1.2g]' % (value, self._range[0], self._range[1]))
                # Also set value for fader
                bcfValue = 127 * (value - self._range[0]) / (self._range[1]-self._range[0]) 
                if self._bcf_fader in range(1,9):
                    BCF.set_fader(self._bcf_fader, int(bcfValue))
                if self._bcf_encoder in range(1,9):
                    BCF.set_encoder(self._bcf_encoder, int(bcfValue))
            finally:
                self._valueChangeFlag = False
            self.onValueChanged()
    
    
    def pushValue(self, val):
        # Automatically change range
        if val < self._range[0]:
            self._setRange(val, self._range[1])
        if val > self._range[1]:
            self._setRange(self._range[0], val)
        # Set range for slider
        self._slider.setValue(val/self.SLIDER_SCALE)
    
    
    def pullValue(self):
        return self._value



class FloatDisplay(ScalarDisplay):
    SLIDER_SCALE = 0.001
    TYPE = float


class IntDisplay(ScalarDisplay):
    SLIDER_SCALE = 1
    TYPE = int


class BoolDisplay(BaseValueDisplay):
    def __init__(self, parent, editable):
        BaseValueDisplay.__init__(self, parent, editable)
        # Create checkbox
        self._w = QtGui.QCheckBox(self)
        self.setMainWidget(self._w)
        #
        self._w.stateChanged.connect(self.onValueChanged)
    
    def pushValue(self, state):
        self._w.setCheckState(state)
    
    def pullValue(self):
        return self._w.checkState()==2



class StrDisplay(BaseValueDisplay):
    def __init__(self, parent, editable):
        BaseValueDisplay.__init__(self, parent, editable)
        # Create line edit
        self._w = QtGui.QLineEdit(self)
        self.setMainWidget(self._w)
        #
        self._w.editingFinished.connect(self.onValueChanged)
        self._w.setReadOnly(not editable)
    
    def pushValue(self, value):
        self._w.setText(value)
    
    def pullValue(self):
        return str(self._w.text())



class TimeDisplay(BaseValueDisplay):
    def __init__(self, parent, editable):
        BaseValueDisplay.__init__(self, parent, editable)
        #
        self._secs = QtGui.QSpinBox(self)
        self._secsLabel = QtGui.QLabel('sec', self)
        self._secs.valueChanged.connect(self.onValueChanged)
        self._secs.setMaximum(99)
        #
        self._nsecs = QtGui.QSpinBox(self)
        self._nsecsLabel = QtGui.QLabel('nsec', self)
        self._nsecs.valueChanged.connect(self._onValueChanged)
        self._nsecs.setMaximum(1000)
        self._nsecs.setMinimum(-1)
        #
        self._secs.setReadOnly(not editable)
        self._nsecs.setReadOnly(not editable)
        #
        layout = QtGui.QHBoxLayout(self)
        layout.setSpacing(2)
        layout.setContentsMargins(0,0,0,0)
        self.setLayout(layout)
        #
        layout.addWidget(self._secs, 0)
        layout.addWidget(self._secsLabel, 0)
        layout.addWidget(self._nsecs, 0)
        layout.addWidget(self._nsecsLabel, 0)
        layout.addStretch(1)
    
    def pushValue(self, value):
        self._secs.setValue(value.secs)
        self._nsecs.setValue(value.nsecs)
    
    def pullValue(self):
        return Time(self._secs.value(), self._nsecs.value())
    
    def _onValueChanged(self):
        # Test if we should roundtrip nsecs and increase/descrease secs
        sec, nsec = self._secs.value(), self._nsecs.value()
        if nsec == 1000:
            if sec < self._secs.maximum():
                self._secs.setValue(sec+1); self._nsecs.setValue(0)
            else:
                self._nsecs.setValue(999)
        elif nsec == -1:
            if sec > 0:
                self._secs.setValue(sec-1); self._nsecs.setValue(999)
            else:
                self._nsecs.setValue(0)
        else:
            # Act normal
            self.onValueChanged()

