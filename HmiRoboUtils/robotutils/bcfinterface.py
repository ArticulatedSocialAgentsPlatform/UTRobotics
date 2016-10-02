#!/usr/bin/env python3
# Copyright (C) 2013, Science Applied v.o.f.

"""
Module for reading and writing to the BCF2000.
"""

try:
    import rtmidi
except ImportError:
    rtmidi = None

BCF_MAGIC_BYTE = 0xB0

BCF_BUTTON_OFFSET = 65  # Buttons have ID 65 -- 80
BCF_BUTTON_COUNT = 16

BCF_ENCODER_OFFSET = 1  # Encoders have ID 1 -- 8
BCF_ENCBUTTON_OFFSET = 33  # Encoder buttons have ID 33 -- 40
BCF_ENCODER_COUNT = 8

BCF_FADER_OFFSET = 81  # Faders have ID 81 -- 88
BCF_FADER_COUNT = 8


class BCFInterface:
    """ BCFInterface(in_port=0, out_port=0)
    
    Provides an interface to the BCF2000 motor fader, by communicating
    MIDI messages with it. Requires the rtmidi package.
    
    Parameters
    ----------
    in_port : int
        Input port on the MIDI interface.
    out_port : int
        Output port on the MIDI interface.
    """
    
    _button_values = [False]*BCF_BUTTON_COUNT
    _encoder_values = [0]*BCF_ENCODER_COUNT
    _encbutton_values = [False]*BCF_ENCODER_COUNT
    _fader_values = [0]*BCF_FADER_COUNT
    
    def __init__(self, in_port=0, out_port=0):        
        # Check rtmidi
        if rtmidi is None:
            raise RuntimeError('BCFInterface requires the rtmidi package.')
        
        # Bind to input
        midi_in = rtmidi.MidiIn()
        midi_in.set_callback(self._data_in)
        midi_in.open_port(in_port)
        print('reading from <%s>' %  midi_in.get_port_name(in_port))
        self._midi_in = midi_in
        
        # Set output
        midi_out = rtmidi.MidiOut()
        midi_out.open_port(out_port)
        print('writing to <%s>' % midi_out.get_port_name(out_port))
        self._midi_out = midi_out
        
        # Set default callbacks
        self._button_callback = None
        self._encoder_callback = None
        self._encbutton_callback = None
        self._fader_callback = None
        
        # Reset controls
        self.reset_all()
    
    
    def _get_button_callback(self):
        """ Callback invoked when one of the pushbuttons is pressed.
        
        This callback is invoked when one of the 16 pushbuttons on the
        top panel of the BCF2000 is pressed. The callback should have
        the following signature:
            
            callable(id, value)
        
        where `id` will be in the range [1, 16] (counting from top-left
        to bottom-right), and `value` will be True (when the button is
        set), or False (when the button is unset).
        """
        return self._button_callback
    
    def _set_button_callback(self, callable):
        self._button_callback = callable
    
    button_callback = property(_get_button_callback,
                               _set_button_callback)
    
    
    def _get_encoder_callback(self):
        """ Callback invoked when one of the rotary encoders is
        manipulated.
        
        This callback is invoked when one of the 8 rotary encoders on
        the top panel of the BCF2000 is manipulated. The callback
        should have the following signature:
            
            callable(id, value)
        
        where `id` will be in the range [1, 8] (counting from left to
        right), and `value` will be in the range of [0, 127].
        """
        return self._encoder_callback
    
    def _set_encoder_callback(self, callable):
        self._encoder_callback = callable
    
    encoder_callback = property(_get_encoder_callback,
                                _set_encoder_callback)
    
    
    def _get_encbutton_callback(self):
        """ Callback invoked when one of the rotary encoders is pushed.
        
        This callback is invoked when one of the 8 rotary encoders on
        the top panel of the BCF2000 is pushed. The callback should have
        the following signature:
            
            callable(id, value)
        
        where `id` will be in the range [1, 8] (counting from top-left
        to bottom-right), and `value` will be True (when the button is
        set), or False (when the button is unset).
        """
        return self._encbutton_callback
    
    def _set_encbutton_callback(self, callable):
        self._encbutton_callback = callable
    
    encbutton_callback = property(_get_encbutton_callback,
                                  _set_encbutton_callback)
    
    
    def _get_fader_callback(self):
        """ Callback invoked when one of the motorized faders is
        manipulated.
        
        This callback is invoked when one of the 8 motorized faders on
        the bottom panel of the BCF2000 is manipulated. The callback
        should have the following signature:
            
            callable(id, value)
        
        where `id` will be in the range [1, 8] (counting from left to
        right), and `value` will be in the range of [0, 127].
        """
        return self._fader_callback
    
    def _set_fader_callback(self, callable):
        self._fader_callback = callable
    
    fader_callback = property(_get_fader_callback,
                              _set_fader_callback)
    
    
    def reset_all(self):
        """ Reset all controls. """
        self.reset_buttons()
        self.reset_encoders()
        self.reset_encbuttons()
        self.reset_faders()
    
    
    def get_button(self, id):
        """ Get the current value for given button (True if set, False
        if unset). Returns None for an invalid id.
        
        Parameters
        ----------
        id : int
            Button-ID within the set [1, BCF_BUTTON_COUNT]
        """
        if id < 1 or id > BCF_BUTTON_COUNT:
            return None
        else:
            return self._button_values[id-1]
    
    
    def set_button(self, id, value=True):
        """ Turn on the button with given ID. If value is False, the
        call is equivalent to unset_button(id)
        
        Parameters
        ----------
        id : int
            Button-ID within the set [1, BCF_BUTTON_COUNT]
        value : bool
            True to set button, False to unset
        """
        if id < 1 or id > BCF_BUTTON_COUNT:
            return
        else:
            self._button_values[id-1] = value
            self._set_value(BCF_BUTTON_OFFSET + id - 1, 127*int(value))
    
    
    def unset_button(self, id):
        """ Turn off the button with given ID.
        
        Parameters
        ----------
        id : int
            Button-ID within the set [1, BCF_BUTTON_COUNT]
        """
        if id < 1 or id > BCF_BUTTON_COUNT:
            return
        else:
            self._button_values[id-1] = False
            self._set_value(BCF_BUTTON_OFFSET + id - 1, 0)
    
    
    def reset_buttons(self):
        """ Turn off all buttons. """
        for i in range(BCF_BUTTON_COUNT):
            id = i + 1
            self.unset_button(id)
    
    
    def get_encoder(self, id):
        """ Get the current value for given encoder. Returns None for
        an invalid id.
        
        Parameters
        ----------
        id : int
            Encoder-ID within the set [1, BCF_ENCODER_COUNT]
        """
        if id < 1 or id > BCF_ENCODER_COUNT:
            return None
        else:
            return self._encoder_values[id-1]
    
    
    def set_encoder(self, id, value):
        """ Set the value for given encoder.
        
        Parameters
        ----------
        id : int
            Fader-ID within the set [1, BCF_ENCODER_COUNT]
        value : int
            Value within range [0, 127]
        """
        
        # Clip value
        if value < 0:
            value = 0
        if value > 127:
            value = 127
        
        # Set value if ID is valid
        if id < 1 or id > BCF_ENCODER_COUNT:
            return
        else:
            self._encoder_values[id-1] = value
            self._set_value(BCF_ENCODER_OFFSET + id - 1, value)
    
    
    def reset_encoders(self):
        """ Reset all faders to zero. """
        for i in range(BCF_ENCODER_COUNT):
            id = i + 1
            self.set_encoder(id, 0)
    
    
    def get_encbutton(self, id):
        """ Get the current value for given encoder button (True if
        set, False if unset). Returns None for an invalid id.
        
        Parameters
        ----------
        id : int
            Encoder-ID within the set [1, BCF_ENCODER_COUNT]
        """
        if id < 1 or id > BCF_ENCODER_COUNT:
            return None
        else:
            return self._encbutton_values[id-1]
    
    
    def set_encbutton(self, id, value=True):
        """ Turn on the encoder button with given ID. If value is False,
        the call is equivalent to unset_encbutton(id)
        
        Parameters
        ----------
        id : int
            Encoder-ID within the set [1, BCF_ENCODER_COUNT]
        value : bool
            True to set button, False to unset
        """
        if id < 1 or id > BCF_ENCODER_COUNT:
            return
        else:
            self._encbutton_values[id-1] = value
            self._set_value(BCF_ENCBUTTON_OFFSET + id - 1, 127*int(value))
    
    
    def unset_encbutton(self, id):
        """ Turn off the encoder button with given ID.
        
        Parameters
        ----------
        id : int
            Encoder-ID within the set [1, BCF_ENCODER_COUNT]
        """
        if id < 1 or id > BCF_ENCODER_COUNT:
            return
        else:
            self._encbutton_values[id-1] = False
            self._set_value(BCF_ENCBUTTON_OFFSET + id - 1, 0)
    
    
    def reset_encbuttons(self):
        """ Turn off all encoder buttons. """
        for i in range(BCF_ENCODER_COUNT):
            id = i + 1
            self.unset_button(id)
    
    
    def get_fader(self, id):
        """ Get the current value for given fader. Returns None for an
        invalid id.
        
        Parameters
        ----------
        id : int
            Fader-ID within the set [1, BCF_FADER_COUNT]
        """
        if id < 1 or id > BCF_FADER_COUNT:
            return None
        else:
            return self._fader_values[id-1]
    
    
    def set_fader(self, id, value):
        """ Set the value for given fader.
        
        Parameters
        ----------
        id : int
            Fader-ID within the set [1, BCF_FADER_COUNT]
        value : int
            Value within range [0, 127]
        """
        
        # Clip value
        if value < 0:
            value = 0
        if value > 127:
            value = 127
        
        # Set value if ID is valid
        if id < 1 or id > BCF_FADER_COUNT:
            return
        else:
            self._fader_values[id-1] = value
            self._set_value(BCF_FADER_OFFSET + id - 1, value)
    
    
    def reset_faders(self):
        """ Reset all faders to zero. """
        for i in range(BCF_FADER_COUNT):
            id = i + 1
            self.set_fader(id, 0)
    
    
    def _data_in(self, message_data, user_data=None):
        data, _ = message_data
        _, ctrl_id, value = data
        
        if ctrl_id in range(BCF_BUTTON_OFFSET,
                            BCF_BUTTON_OFFSET + BCF_BUTTON_COUNT):
            # Push buttons
            id = ctrl_id - BCF_BUTTON_OFFSET + 1
            self._button_values[id - 1] = bool(value)
            if self.button_callback:
                self.button_callback(id, bool(value))
        elif ctrl_id in range(BCF_ENCODER_OFFSET,
                              BCF_ENCODER_OFFSET + BCF_ENCODER_COUNT):
            # Rotary encoders
            id = ctrl_id - BCF_ENCODER_OFFSET + 1
            self._encoder_values[id - 1] = value
            if self.encoder_callback:
                self.encoder_callback(id, value)
        elif ctrl_id in range(BCF_ENCBUTTON_OFFSET,
                              BCF_ENCBUTTON_OFFSET + BCF_ENCODER_COUNT):
            # Encoder buttons
            id = ctrl_id - BCF_ENCBUTTON_OFFSET + 1
            self._encbutton_values[id - 1] = bool(value)
            if self.encbutton_callback:
                self.encbutton_callback(id, bool(value))
        elif ctrl_id in range(BCF_FADER_OFFSET,
                              BCF_FADER_OFFSET + BCF_FADER_COUNT):
            # Faders
            id = ctrl_id - BCF_FADER_OFFSET + 1
            self._fader_values[id - 1] = value
            if self.fader_callback:
                self.fader_callback(id, value)
    
    
    def _set_value(self, id, value):
        message = [BCF_MAGIC_BYTE, id, value]
        self._midi_out.send_message(message)

    
    def dance(self, timeout=3.0):
        import math, time
        timeout = time.time() + timeout
        while time.time() < timeout:
            time.sleep(0.01)
            t = time.time() * 5
            
            for j in range(1,9):
                phase = j*math.pi / 4
                v = math.cos(t+phase)
                self.set_fader(j, v*64+64)
                self.set_encoder(j, v*64+64)

        

if __name__ == '__main__':
    bcf = BCFInterface()
    
    def set_partner(id, value, set_control):
        pairs = {1: 8, 2: 7, 3: 6, 4: 5, 9: 16, 10: 15, 11: 14, 12: 13}
        for p1, p2 in pairs.items():
            if p1 == id:
                target_id = p2
                break
            elif p2 == id:
                target_id = p1
                break
            else:
                continue
        
        set_control(target_id, value)
    
    bcf.button_callback = lambda id, value: set_partner(id, value, bcf.set_button)
    bcf.encoder_callback = lambda id, value: set_partner(id, value, bcf.set_encoder)
    bcf.encbutton_callback = lambda id, value: set_partner(id, value, bcf.set_encbutton)
    bcf.fader_callback = lambda id, value: set_partner(id, value, bcf.set_fader)
    
    bcf.dance()
    