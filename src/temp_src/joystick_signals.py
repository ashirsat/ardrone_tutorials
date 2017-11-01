# Import PyQt4 Libraries
from PyQt4 import QtCore


class ControllerSignals(QtCore.QObject):
    """ Emits signals to GUI.

    Test variable: Will add more soon.

    Signals:
        L_STICK_HORZ:
        L_STICK_VERT:
        R_STICK_HORZ:
        R_STICK_VERT:
        D_PAD_UP:
        D_PAD_DOWN:
        D_PAD_LEFT:
        D_PAD_RIGHT:
        B_BUTTON:
        X_BUTTON:
        Y_BUTTON:
        A_BUTTON:
        START:
        BACK:
        LB:
        LT:
        RB:
        RT:
    """
    # Left Stick Motion:
    L_STICK_HORZ = QtCore.pyqtSignal(int, int, int)
    L_STICK_VERT = QtCore.pyqtSignal(int, int, int)

    # Right Stick  Motion:
    R_STICK_HORZ = QtCore.pyqtSignal(int, int, int)
    R_STICK_VERT = QtCore.pyqtSignal(int, int, int)

    # D-Pad Motion:
    D_PAD_UP = QtCore.pyqtSignal(int, int)
    D_PAD_DOWN = QtCore.pyqtSignal(int, int)
    D_PAD_LEFT = QtCore.pyqtSignal(int, int)
    D_PAD_RIGHT = QtCore.pyqtSignal(int, int)

    # Joystick Buttons:
    B_BUTTON = QtCore.pyqtSignal(int, int)
    X_BUTTON = QtCore.pyqtSignal(int, int)
    Y_BUTTON = QtCore.pyqtSignal(int)
    A_BUTTON = QtCore.pyqtSignal(int)

    # Option Buttons:
    START = QtCore.pyqtSignal(int)
    BACK = QtCore.pyqtSignal(int)

    # Bumper Buttons:
    LB = QtCore.pyqtSignal(int, int)
    LT = QtCore.pyqtSignal(int, int)
    RB = QtCore.pyqtSignal(int, int)
    RT = QtCore.pyqtSignal(int, int)

    def __init__(self):
        QtCore.QObject.__init__(self)
