from PyQt4 import QtCore, QtGui
import sys
import cv2
class MainGUI(QtGui.QMainWindow):
    def __init__(self):
        super(MainGUI, self).__init__()
        self.initUI()

    def initUI(self):

        self.wid = QtGui.QWidget()
        l1 = QtGui.QLabel()
        l2 = QtGui.QLabel()
        # img = "/home/aniketrs/ardrone_catkin_ws/src/ardrone_tutorials/src/Testruns_videos/output1_test1.avi"
        # img2 = "/home/aniketrs/ardrone_catkin_ws/src/ardrone_tutorials/src/Testruns_videos/output2_test1.avi"
        img = "/home/aniketrs/ardrone_catkin_ws/src/ardrone_tutorials/src/Test_images/TestRun3/image1a1.png"
        img2 = "/home/aniketrs/ardrone_catkin_ws/src/ardrone_tutorials/src/Test_images/TestRun3/image2b1.png"
        # print img.shape
        prof1 = QtGui.QImage()
        prof1.load(img)
        prof1.load(img2)
        print (prof1.load(img))
        print (prof1.load(img2))

        # prof1 = prof1.scaled(360, 640, aspectRatioMode=QtCore.Qt.IgnoreAspectRatio)
        # l2 = QtGui.QLabel()
        pixmap1 = QtGui.QPixmap(img)
        pixmap2 = QtGui.QPixmap(img2)
        # pixmap = "/home/aniketrs/ardrone_catkin_ws/src/ardrone_tutorials/src/Test_images/TestRun3/image1a1.png"
        l1.setAlignment(QtCore.Qt.AlignLeft)
        l1.setPixmap(pixmap1)
        l2.setAlignment(QtCore.Qt.AlignRight)
        l2.setPixmap(pixmap2)
        # l1.setMask(pixmap.mask())
        # r1 = QtGui.QImageReader()
        # l1.setPixmap(QtGui.QImage(("/home/aniketrs/ardrone_catkin_ws/src/ardrone_tutorials/src/Test_images/TestRun3/image11.jpg")))
        # l2.setPixmap(QtGui.QPixmap('/home/aniketrs/ardrone_catkin_ws/src/ardrone_tutorials/src/Test_images/TestRun3/img52.jpg'))

        hbox = QtGui.QHBoxLayout()
        hbox.addWidget(l1)
        hbox.addWidget(l2)
        self.wid.setLayout(hbox)
        self.wid.update()

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    # app.addLibraryPath(r"/home/aniketrs/ardrone_catkin_ws/src/ardrone_tutorials/src/Test_images/TestRun3/")
    display = MainGUI()
    # wid1 = ImgWidget1()
    display.wid.show()
    # display.show()
    status = app.exec_()
    sys.exit(status)