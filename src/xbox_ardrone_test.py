import xbox
from threading import Thread, Lock, ThreadError
# import msvcrt
import sys

def Xboxjoystick(joy):
    if joy.A():
        print 'A Pressed'
    if joy.leftStick():
        print joy.leftStick()
    # joy.close()

# try:
#     thred.start()
# except ThreadError:
# finally:

# joy = xbox.Joystick()
if __name__ == '__main__':
    joy = xbox.Joystick()
    xbox=Xboxjoystick(joy)
    thred = Thread()
    try:
        thred.start()
    except (KeyboardInterrupt, SystemExit):
        thred.join(timeout=10.0)
        sys.exit()
    # if msvcrt.kbhit():
    #     key = ord(msvcrt.getch())
    #     if key == 27:
    #         break
    # # if
# import pygame
# import threading
#
# lock = threading.Lock()
# pygame.init()
#
# # pygame.joystick.init()
#
# done = False
# clock = pygame.time.Clock()
# lock.acquire()
# threading.Thread.start(name=None,)
# try:
#     if threading.Thread.start() is True:
# # while done == False:
#         stick_key = pygame.event.get()
#         for x in stick_key:
#             print x
#             if x.type == pygame.QUIT:
#         # done = True
#                 print 'Quiting'
#             if x.type == pygame.JOYBUTTONDOWN and x.dict['button'] == 1:
#                 print "Joystick button pressed"
#             # print pygame.JOYBUTTONDOWN#event_name(event.type)
#         # if event.type == pygame.JOYBUTTONUP:
#         #     print "Joystick button released"
#
#         joystick_count = pygame.joystick.get_count()
#
#     # for i in range(joystick_count):
#         joystick = pygame.joystick.Joystick(1)
#         joystick.init()
#
#         axes = joystick.get_numaxes()
#         # buttons = joystick.get_numbuttons()
#         button1 = joystick.get_button(0)
#         if button1 is pygame.JOYBUTTONDOWN:
#             print 'A down'
#     # for i in range(axes):
#     # axis = joystick.get_axis(0)
#     # print axis
#     # print "event type" %pygame.event.get()
#     # print
#     #
#     clock.tick(20)
# finally:
#     lock.release()
#     pygame.quit()
#
