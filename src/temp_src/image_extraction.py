import cv2
import os
import glob
cap1 = cv2.VideoCapture('/home/aniketrs/ardrone_catkin_ws/src/ardrone_tutorials/src/Testruns_videos/output1_test3.avi')
cap2 = cv2.VideoCapture('/home/aniketrs/ardrone_catkin_ws/src/ardrone_tutorials/src/Testruns_videos/output2_test3.avi')

ret1, frames1 = cap1.read()
ret2, frames2 = cap2.read()
frame_ct1 = cap1.get(5)
frame_ct2 = cap2.get(5)
print '#Frames1=%s, Frames2=%s' %(frame_ct1, frame_ct2)
count = 0
ret1 = True
ret2 = True
fr1 = 0
fr2 = 0
while ret1 is True and ret2 is True:
    if fr1 <= 10:
        ret1, frames1 = cap1.read()
        # print 'skipping 1 Quad stream:%s ', fr1
        ret2, frames2 = cap2.read()
        # print 'skipping 2 Quad stream %s', fr1
        fr1 += 1
    else:
        ret1, frames1 = cap1.read()
        print 'Writing quad1:%d'%count
        cv2.imwrite('/home/aniketrs/ardrone_catkin_ws/src/ardrone_tutorials/src/Test_images/TestRun3/global/image1a%d.png' %(count + 1),
                frames1)
        print 'Writing Quad2 %d'%count
        cv2.imwrite('/home/aniketrs/ardrone_catkin_ws/src/ardrone_tutorials/src/Test_images/TestRun3/global/image2b%d.png' %(count + 1),
                frames2)
        fr1 = 0
        count += 1
    # count += 1


cap1.release()
cap2.release()


# count=1
# files = glob.glob('/home/aniketrs/ardrone_catkin_ws/src/ardrone_tutorials/src/Test_images/TestRun3/*.jpg')
# with open('/home/aniketrs/ardrone_catkin_ws/src/ardrone_tutorials/src/Test_images/TestRun3/imagelist3.txt', 'w') as in_files:
#     for each_file in files:
#         in_files.write(each_file+'\n')
#         count += 1
#         print 'writing'