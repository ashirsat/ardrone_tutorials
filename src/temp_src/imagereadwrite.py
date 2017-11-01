import os
import glob
count=1
files = glob.glob('/home/aniketrs/ardrone_catkin_ws/src/ardrone_tutorials/src/Test_images/TestRun3/*.jpg')
with open('/home/aniketrs/ardrone_catkin_ws/src/ardrone_tutorials/src/Test_images/TestRun3/imagelist3.txt', 'w') as in_files:
    for each_file in files:
        in_files.write(each_file+'\n')
        count += 1
        print 'writing'


# for filename in os.listdir('/home/aniketrs/ardrone_catkin_ws/src/ardrone_tutorials/src/Test_images/TestRun5'):
# os.path.join('/home/aniketrs/ardrone_catkin_ws/src/ardrone_tutorials/src/Test_images/TestRun5', filename)
