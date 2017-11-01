import sys
import cv2
from PyQt4 import QtCore, QtGui

class MyDialog(QtGui.QMainWindow):
    def __init__(self, parent=None):
        super(MyDialog, self).__init__(parent)

        self.left = 700
        self.top = 400
        self.width = 1920
        self.height = 1080

        self.setWindowTitle('Sender')
        # self.setGeometry(self.left, self.top, self.width, self.height)
        # self.setFixedSize(1280, 1080)

        self.videoFrame = ImageWidget()
        self.videoFrame2 = ImageWidget2()
        self.label1 = QtGui.QLabel(self.videoFrame)
        self.label2 = QtGui.QLabel(self.videoFrame2)

        widget = QtGui.QWidget(self)
        layout = QtGui.QHBoxLayout(widget)
        self.videoFrame.setGeometry(0, 0, 640, 360)
        self.videoFrame2.setGeometry(650, 0, 640, 360)
        layout.addWidget(self.videoFrame)
        layout.addWidget(self.videoFrame2)
        self.setCentralWidget(widget)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.updateImage)
        self.timer.start(30)
        # self.capture1 = cv2.VideoCapture(0)
        # self.capture2 = cv2.VideoCapture(0)

    def updateImage(self):
        # _, img = self.capture1.read()
        # height, width, bpc = img.shape
        # bpl = bpc * width
        width = 360
        height = 640
        img = "/home/aniketrs/ardrone_catkin_ws/src/ardrone_tutorials/src/Test_images/TestRun3/image1.jpg"
        # image = QtGui.QImage(img, width, height, QtGui.QImage.Format_RGB888)
        pixmap2 = QtGui.QPixmap(img)#.scaled(360, 640,aspectRatioMode=QtCore.Qt.IgnoreAspectRatio)# image.scaled(300, 300, QtCore.Qt.KeepAspectRatio)
        pixmap2.scaledToHeight(360)
        pixmap2.scaledToWidth(640)
        self.resize(width, height)
        self.label1.setPixmap(pixmap2)

        # _, img0 = self.capture2.read()
        # height, width, bpc = img0.shape
        # bpl = bpc * width
        height = 360
        width = 640
        img0 = "/home/aniketrs/ardrone_catkin_ws/src/ardrone_tutorials/src/Test_images/TestRun3/image331.jpg"
        # image2 = QtGui.QImage(img0, width, height, QtGui.QImage.Format_RGB888)
        pixmap2 = QtGui.QPixmap(img0)#.scaled(360, 640,aspectRatioMode=QtCore.Qt.IgnoreAspectRatio)
        # pixmap2 = image2.scaled(300, 300, QtCore.Qt.KeepAspectRatio)
        pixmap2.scaledToHeight(360)
        pixmap2.scaledToWidth(640)
        self.resize(width, height)
        self.label2.setPixmap(pixmap2)


class ImageWidget(QtGui.QWidget):
    def __init__(self, parent=None):
        super(ImageWidget, self).__init__(parent)
        self.image = None

    def setImage(self, image):
        self.image = image
        # sz = image.size()
        # self.setMinimumSize(360, 640)
        # self.pixmap = QtGui.QPixmap(self.image)
        self.update()


class ImageWidget2(QtGui.QWidget):
    def __init__(self, parent=None):
        super(ImageWidget2, self).__init__(parent)
        self.image = None

    def setImage(self, image):
        self.image = image
        # sz = image.size()
        # self.setMinimumSize(360, 640)
        self.update()

# class Image_gui(QtGui.QMainWindow):
#     def __init__(self):
#         super(Image_gui, self).__init__()
#         self.initGUI()
#         self.img1 = ImageWidget()   #None #"/home/aniketrs/ardrone_catkin_ws/src/ardrone_tutorials/src/Test_images/TestRun3/image_1.jpg"
#         self.img2 = ImageWidget()   #None #"/home/aniketrs/ardrone_catkin_ws/src/ardrone_tutorials/src/Test_images/TestRun3/image_2.jpg"
#
#     def initGUI(self):
#         self.setWindowTitle('Image Analysis')
#         img_widget = QtGui.QWidget(self)
#         hbox = QtGui.QHBoxLayout(img_widget)
#         self.img1.setGeometry(0, 0, 640, 360)
#         self.img2.setGeometry(650, 0, 640, 360)
#         self.img1.image = "/home/aniketrs/ardrone_catkin_ws/src/ardrone_tutorials/src/Test_images/TestRun3/image1.jpg"
#         self.img2.image = "/home/aniketrs/ardrone_catkin_ws/src/ardrone_tutorials/src/Test_images/TestRun3/image331.jpg"
#         hbox.addWidget(self.img1)
#         hbox.addWidget(self.img2)
#         self.setCentralWidget(img_widget)
#         # self.box1 = QtGui.QLabel(self)
#         # self.box2 = QtGui.QLabel(self)
#         # centralwidget = QtGui.QWidget()
#         # image1 = QtGui.QWidget()
#         # image2 = QtGui.QWidget()
#         # hbox1 = QtGui.QHBoxLayout(centralwidget)
#         # # hbox2 = QtGui.QHBoxLayout()
#         # hbox1.addWidget(image1)
#         # hbox1.addWidget(image2)
#         # self.setCentralWidget(centralwidget)
#         # image1.setLayout(hbox1)
#         # image2.setLayout(hbox2)
#         # # self.setCentralWidget(self.box1)
#         # # self.setCentralWidget(self.box2)
#         # image1.setGeometry(0, 0, 640, 360)
#         # image2.setGeometry(650, 0, 640, 360)
#         # self.img1 = "/home/aniketrs/ardrone_catkin_ws/src/ardrone_tutorials/src/Test_images/TestRun3/image_1.jpg"
#         # self.img2 = "/home/aniketrs/ardrone_catkin_ws/src/ardrone_tutorials/src/Test_images/TestRun3/image_2.jpg"
#         # cvimg1 = QtGui.QPixmap(self.img1)
#         # cvimg2 = QtGui.QPixmap(self.img2)
#         # self.box1.setPixmap(cvimg1)
#         # self.box2.setPixmap(cvimg2)
#
# class ImageWidget(QtGui.QWidget):
#     def __init__(self):
#         super(ImageWidget, self).__init__()
#         self.image = None
#         self.disp_image(self, self.image)
#         self.cvimg = None
#
#     def disp_image(self, data):
#         self.imageBox = QtGui.QLabel(self)
#         self.image = data
#         self.width = 640
#         self.height = 360
#         self.update()
#         if self.image is not None:
#             try:
#                 self.cvimg = QtGui.QPixmap(QtGui.QImage(self.image, self.width, self.height, QtGui.QImage.Format_RGB888))
#             except KeyboardInterrupt, e:
#                 print "exited due to error %s"%e
#             self.imageBox.setPixmap(self.cvimg)

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    gui = MyDialog()
    imag = ImageWidget()
    imag2 = ImageWidget2()
    gui.show()
    status = app.exec_()
    sys.exit(status)



