
import sys

# Import ...
if 'PySide' in sys.modules:
    from PySide import QtGui, QtCore
elif 'PyQt4' in sys.modules:
    from PyQt4 import QtGui, QtCore
else:
    try:
        from PySide import QtGui, QtCore
    except ImportError:
        try:
            from PyQt4 import QtGui, QtCore
        except ImportError:
            raise ImportError("Both PySide and PyQt4 could not be imported.")

# Make PyQt4 more PySide-ish
if hasattr(QtCore, 'PYQT_VERSION'):
    QtCore.Signal = QtCore.pyqtSignal

