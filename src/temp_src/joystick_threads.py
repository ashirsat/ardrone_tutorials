# Import System Libraries
import sys
import threading
import time

# Import pyusb Libraries
import usb.core
import usb.util

# Import other Libraries
import joystick_signals


class BlackJoystickThread(threading.Thread):
    """ Test Controller Thread.

    A thread for the black controller functions.

    Attributes:
        device:

    """
    def __init__(self):
        threading.Thread.__init__(self)

        # Stopping Mechanism
        self._stop = threading.Event()

        # A 1x4 list containing which button is pressed and at what value it is
        # currently at.
        self.activated = True

        # Subject Number.
        self.subject_number = 1 ## What is subject number for ?

        # Instantiate the ControllerSignals class
        self.signal = joystick_signals.ControllerSignals()

        # Select the device.
        self.device = usb.core.find(
            idVendor=0x45e, idProduct=0x28e)#, address=0x003)

        if self.device is None:
            raise ValueError('Device not found.')

        # Detach device from kernel driver (FOR LINUX ONLY)
        for cfg in self.device:
            for intf in cfg:
                if self.device.is_kernel_driver_active(intf.bInterfaceNumber):
                    try:
                        self.device.detach_kernel_driver(intf.bInterfaceNumber)
                        usb.util.claim_interface(self.device, intf.bInterfaceNumber)
                    except:
                        sys.exit("Could not detach kernel drive.")

        # Allow to our new driver
        # usb.util.claim_interface(self.device, 0)
        print('Black joystick thread created.')

    def run(self):
        """ Thread RUN call. """
        print('Black joystick thread started.')
        while self.activated:
            # button[0] = Horizontal R_Stick  (L) 0 ==> 255 (R)
            # button[1] = Vertical R_Stick    (U) 0 ==> 255 (D)
            # button[2] = Horizontal L_Stick  (L) 0 ==> 255 (R)
            # button[3] = Vertical L_Stick    (U) 0 ==> 255 (D)
            # button[5] = D-Pad and Buttons
            # button[6] = Bumper and Option Buttons
            button = self.device.ctrl_transfer(0xa1, 0x1, 0x0101, 0, 0x31) ## what does this do?
    
            # New Method #
            time.sleep(0.2)

            # Horizontal R-Stick
            if button[0] != 128:
                self.signal.R_STICK_HORZ.emit(
                    0, (button[0] / 10), self.subject_number)

            # Vertical R-Stick
            elif button[1] != 128:
                self.signal.R_STICK_VERT.emit(
                    1, (button[1] / 10), self.subject_number)

            # Horizontal L-Stick
            elif button[2] != 128:
                self.signal.L_STICK_HORZ.emit(
                    2, (button[2] / 10), self.subject_number)

            # Vertical L-Stick
            elif button[3] != 128:
                self.signal.L_STICK_VERT.emit(
                    3, (button[3] / 10), self.subject_number)

            # D-Pad Up
            elif button[5] == 0:
                self.signal.D_PAD_UP.emit(4, self.subject_number)

            # D-Pad Down
            elif button[5] == 4:
                self.signal.D_PAD_DOWN.emit(5, self.subject_number)

            # D-Pad Left
            elif button[5] == 6:
                self.signal.D_PAD_LEFT.emit(6, self.subject_number)

            # D-Pad Right
            elif button[5] == 2:
                self.signal.D_PAD_RIGHT.emit(7, self.subject_number)

            # Triangle Button
            elif button[5] == 31:
                self.signal.TRIANGLE.emit(8)

            # X Button
            elif button[5] == 79:
                self.signal.X_MARK.emit(9)

            # Circle Button
            elif button[5] == 47:
                self.signal.CIRCLE.emit(10, self.subject_number)

            # Square Button
            elif button[5] == 143:
                self.signal.SQUARE.emit(11, self.subject_number)

            # L1 Button
            elif button[6] == 1:
                self.signal.L1.emit(12, self.subject_number)

            # L2 Button
            elif button[6] == 4:
                self.signal.L2.emit(13, self.subject_number)

            # R1 Button
            elif button[6] == 2:
                self.signal.R1.emit(14, self.subject_number)

            # R2 Button
            elif button[6] == 8:
                self.signal.R2.emit(15, self.subject_number)

            # Start Button
            elif button[6] == 32:
                self.signal.START.emit(16)

            # Select Button
            elif button[6] == 16:
                self.signal.SELECT.emit(17)

        print('Black joystick thread has stopped.')
        return

    # Remove eventually
    def stop(self):
        self._stop.set()


class BlueJoystickThread(threading.Thread):
    """ Test Controller Thread.

    A thread for the blue controller functions.

    Attributes:
        device:

    """
    def __init__(self):
        threading.Thread.__init__(self)

        # Stopping Mechanism
        self._stop = threading.Event()

        # A 1x4 list containing which button is pressed and at what value it is
        # currently at.
        self.activated = True

        # Subject Number.
        self.subject_number = 2

        # Instantantiate the ControllerSignals class
        self.signal = joystick_signals.ControllerSignals()

        # Select the device.
        self.device = usb.core.find(idVendor=0x0e8f, idProduct=0x0003, address=0x002) ## Xbox controller ids

        if self.device is None:
            raise ValueError('Device not found.')

        # Detach device from kernel driver (FOR LINUX ONLY)
        for cfg in self.device:
            for intf in cfg:
                if self.device.is_kernel_driver_active(intf.bInterfaceNumber):
                    try:
                        self.device.detach_kernel_driver(intf.bInterfaceNumber)
                        usb.util.claim_interface(self.device, intf.bInterfaceNumber)
        
                    except:
                        sys.exit("Could not detach kernel drive.")

        # Allow to our new driver
        # usb.util.claim_interface(self.device, 0)
        print('Blue joystick thread created.')

    def run(self):
        """ Thread RUN call. """
        print('Blue joystick thread started.')
        while self.activated:
            # button[0] = Horizontal R_Stick  (L) 0 ==> 255 (R)
            # button[1] = Vertical R_Stick    (U) 0 ==> 255 (D)
            # button[2] = Horizontal L_Stick  (L) 0 ==> 255 (R)
            # button[3] = Vertical L_Stick    (U) 0 ==> 255 (D)
            # button[5] = D-Pad and Buttons
            # button[6] = Bumper and Option Buttons
            button = self.device.ctrl_transfer(0xa1, 0x1, 0x0101, 0, 0x31) ## What does this do?

            # New Method #
            time.sleep(0.2)

            # Horizontal R-Stick
            if button[0] != 128:
                self.signal.R_STICK_HORZ.emit(0, (button[0] / 10), self.subject_number)

            # Vertical R-Stick
            elif button[1] != 128:
                self.signal.R_STICK_VERT.emit(1, (button[1] / 10), self.subject_number)

            # Horizontal L-Stick
            elif button[2] != 128:
                self.signal.L_STICK_HORZ.emit(2, (button[2] / 10), self.subject_number)

            # Vertical L-Stick
            elif button[3] != 128:
                self.signal.L_STICK_VERT.emit(3, (button[3] / 10), self.subject_number)

            # D-Pad Up
            elif button[5] == 0:
                self.signal.D_PAD_UP.emit(4, self.subject_number)

            # D-Pad Down
            elif button[5] == 4:
                self.signal.D_PAD_DOWN.emit(5, self.subject_number)

            # D-Pad Left
            elif button[5] == 6:
                self.signal.D_PAD_LEFT.emit(6, self.subject_number)

            # D-Pad Right
            elif button[5] == 2:
                self.signal.D_PAD_RIGHT.emit(7, self.subject_number)

            # Y Button
            elif button[5] == 31:
                self.signal.Y_BUTTON.emit(8)

            # A Button
            elif button[5] == 79:
                self.signal.A_BUTTON.emit(9)

            # B Button
            elif button[5] == 47:
                self.signal.B_BUTTON.emit(10, self.subject_number)

            # X Button
            elif button[5] == 143:
                self.signal.X_BUTTON.emit(11, self.subject_number)

            # LB Button
            elif button[6] == 1:
                self.signal.LB.emit(12, self.subject_number)

            # LT Button
            elif button[6] == 4:
                self.signal.LT.emit(13, self.subject_number)

            # RB Button
            elif button[6] == 2:
                self.signal.RB.emit(14, self.subject_number)

            # RT Button
            elif button[6] == 8:
                self.signal.RT.emit(15, self.subject_number)

            # Start Button
            elif button[6] == 32:
                self.signal.START.emit(16)

            # Back Button
            elif button[6] == 16:
                self.signal.BACK.emit(17)

        print('Blue joystick thread has stopped.')
        return

    # Remove eventually
    def stop(self):
        self._stop.set()
