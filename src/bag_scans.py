#!/usr/bin/env python

import sys, os
import numpy as np
from datetime import datetime, timedelta
from PIL import ImageFile
import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy
import rospkg
from ros import rosbag
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

class MyLaserScan():
    def __init__(self,arr):
        self.id = int(arr[0])
        self.label = arr[1]
        self.sensor_pose_x = float(arr[2])
        self.sensor_pose_y = float(arr[3])
        self.sensor_pose_z = float(arr[4])
        self.sensor_pose_yaw   = float(arr[5])
        self.sensor_pose_pitch = float(arr[6])
        self.sensor_pose_roll  = float(arr[7])
        self.time_stamp = float(arr[8])
        self.date = datetime(1601, 1, 1)+timedelta(microseconds=self.time_stamp*0.1)
        self.time_stamp = (self.date - datetime(1970, 1, 1)).total_seconds()

    def readScan(self,filename):
        c = 0
        for line in open(filename,"r"):
            z = line.split()
            if(z[0]!='#'):
                if c==0:
                    self.aperture = float(z[0])
                elif c==1:
                    self.max_range = float(z[0])
                elif c==2:
                    self.number_of_scans = int(z[0])
                elif c==3:
                    self.scans = []
                    for measurement in z:
                        self.scans.append(float(measurement))
                elif c==4:
                    self.valid = []
                    for v in z:
                        self.valid.append(bool(v))
                c += 1

    def __str__(self):
        return "id: "+str(self.id)+" label: "+self.label+" numReadings: "+str(self.number_of_scans)+" time: "+str(self.date)

class MyRGBD():
    def __init__(self,arr):
        self.id = int(arr[0])
        self.label = arr[1]
        self.sensor_pose_x = float(arr[2])
        self.sensor_pose_y = float(arr[3])
        self.sensor_pose_z = float(arr[4])
        self.sensor_pose_yaw   = float(arr[5])
        self.sensor_pose_pitch = float(arr[6])
        self.sensor_pose_roll  = float(arr[7])
        self.time_stamp = float(arr[8])
        self.date = datetime(1601, 1, 1)+timedelta(microseconds=self.time_stamp*0.1)
        self.time_stamp = (self.date - datetime(1970, 1, 1)).total_seconds()

        self.bridge = CvBridge()

    def read_image_PIL(self,filename):
        print filename
        fp = open(filename, "rb")
        p = ImageFile.Parser()
        while 1:
            s = fp.read(1024)
            if not s:
                break
            p.feed(s)
        im = p.close()
#        print im.format, im.mode, im.size, im.width, im.height, im.info

#        imrot = im.rotate(90)
#        im = imrot

        im_rgb = Image()

        im_rgb.height = im.height    # image height, that is, number of rows
        im_rgb.width  = im.width     # image width, that is, number of columns

        # uint8[] data   --- actual matrix data, size is (step * rows)
        im_rgb.data = [pix for pixdata in image_data_rgb.getdata() for pix in pixdata]

        return im_rgb

    def read_image_CV(self,filename):
        print filename
        if filename.find('depth') != -1:
#            cv_image = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
            cv_image = cv2.imread(filename, -1)
        else:
            cv_image = cv2.imread(filename, cv2.IMREAD_COLOR)
#        cv_image = np.rot90(cv_image).copy() 

        if filename.find('intensity') != -1:
            cv2.imshow('image',cv_image)
            cv2.waitKey(10)
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        image_message = self.bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
        return image_message

    def __str__(self):
        return "id: "+str(self.id)+" label: "+self.label+" time: "+str(self.date)


class Input():
    fullDir = None
    dir = None
    type = None
    datasetName = None
    roomName = None

    def __init__(self,args):
        self.fulldir = args[0]

        # pos = -1
        p=self.fulldir.find('laser_scans')
        if(p != -1):
            # pos = p
            self.type = 'LASER'
        else:
            p=self.fulldir.find('rgbd_data')
            if(p != -1):
                # pos = p
                self.type = 'RGBD'

        rospack = rospkg.RosPack()
        self.dir = rospack.get_path('robotathome_at_ros')+"/bagfiles/"

        strs = self.fulldir.split('/')
        if strs[-1]=='':
            self.datasetName = strs[-3]
            self.roomName = strs[-2]
        else:
            self.datasetName = strs[-2]
            self.roomName = strs[-1]

        # self.datasetName = os.path.basename(os.path.normpath(self.fulldir[:pos]))
        # self.roomName = os.path.basename(os.path.normpath(self.fulldir[pos:]))

    def printAll(self):
        print "FULLDIR:",self.fulldir
        print "DIR:",self.dir
        print "TYPE:",self.type
        print "DATASET NAME:",self.datasetName
        print "ROOM NAME:",self.roomName

    def createBagOfLaserScans(self,sensorName,readings):
        bagName = self.datasetName+'_'+self.roomName+'_'+self.type+'-'+sensorName+'.bag'
        print self.dir+bagName

        if not os.path.exists(self.dir+self.datasetName+'/'+self.roomName+'/'):
            os.makedirs(self.dir+self.datasetName+'/'+self.roomName+'/')

        bag =rosbag.Bag(self.dir+self.datasetName+'/'+self.roomName+'/'+bagName, 'w')

        try:
            for r in readings:
                print("Adding %i" % r.id)

                ls = LaserScan()
                ls.header.seq = r.id
                ls.header.stamp = rospy.Time(r.time_stamp)
                ls.header.frame_id = 'laser' #r.label

                # start angle of the scan [rad]
                ls.angle_min = -r.aperture/2.0
                # end angle of the scan [rad]
                ls.angle_max =  r.aperture/2.0
                # angular distance between measurements [rad]
                ls.angle_increment = r.aperture / r.number_of_scans

                # time between scans [seconds]
                ls.scan_time = 0.1
                # time between measurements [seconds] - if your scanner
                # is moving, this will be used in interpolating position
                # of 3d points
                ls.time_increment = ls.scan_time/r.number_of_scans

                # minimum range value [m]
                ls.range_min=0
                # maximum range value [m]
                ls.range_max = r.max_range

                # range data [m] (Note: values < range_min or > range_max should be discarded)
                ls.ranges = r.scans

                # intensity data [device-specific units].  If your
                # device does not provide intensities, please leave
                # the array empty.
                ls.intensities = []           

                print ls

                bag.write('scan', ls, ls.header.stamp)
        finally:
            bag.close()

    def getCameraInfo(self):
        camInfo = CameraInfo()

        cx = 157.3245865
        cy = 120.0802295
        fx = 286.441384
        fy = 271.36999

        # The image dimensions with which the camera was calibrated. Normally
        # this will be the full camera resolution in pixels.
        camInfo.height = 240
        camInfo.width = 320

        # The distortion model used. Supported models are listed in
        # sensor_msgs/distortion_models.h. For most cameras, "plumb_bob" - a
        # simple model of radial and tangential distortion - is sufficient.
        camInfo.distortion_model = "plumb_bob"

        # The distortion parameters, size depending on the distortion model.
        # For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
#       camInfo.D = [0, 0, 0, 0, 0]

        # Intrinsic camera matrix for the raw (distorted) images.
        #     [fx  0 cx]
        # K = [ 0 fy cy]
        #     [ 0  0  1]
        # Projects 3D points in the camera coordinate frame to 2D pixel
        # coordinates using the focal lengths (fx, fy) and principal point (cx, cy).
        camInfo.K = [fx,0,cx,0,fy,cy,0,0,1] # 3x3 row-major matrix

        # Rectification matrix (stereo cameras only)
        # A rotation matrix aligning the camera coordinate system to the ideal
        # stereo image plane so that epipolar lines in both stereo images are parallel.
        camInfo.R = [1,0,0,0,1,0,0,0,1] # 3x3 row-major matrix

        # Projection/camera matrix
        #     [fx'  0  cx' Tx]
        # P = [ 0  fy' cy' Ty]
        #     [ 0   0   1   0]
        # By convention, this matrix specifies the intrinsic (camera) matrix
        #  of the processed (rectified) image. That is, the left 3x3 portion
        #  is the normal camera intrinsic matrix for the rectified image.
        # It projects 3D points in the camera coordinate frame to 2D pixel
        #  coordinates using the focal lengths (fx', fy') and principal point
        #  (cx', cy') - these may differ from the values in K.
        # For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will
        #  also have R = the identity and P[1:3,1:3] = K.
        camInfo.P = [fx,0,cx,0,0,fy,cy,0,0,0,1,0] # 3x4 row-major matrix

        # Binning refers here to any camera setting which combines rectangular
        #  neighborhoods of pixels into larger "super-pixels." It reduces the
        #  resolution of the output image to
        #  (width / binning_x) x (height / binning_y).
        # The default values binning_x = binning_y = 0 is considered the same
        #  as binning_x = binning_y = 1 (no subsampling).
#        camInfo.binning_x
#        camInfo.binning_y

        # Region of interest (subwindow of full camera resolution), given in
        #  full resolution (unbinned) image coordinates. A particular ROI
        #  always denotes the same window of pixels on the camera sensor,
        #  regardless of binning settings.
        # The default setting of roi (all values 0) is considered the same as
        #  full resolution (roi.width = width, roi.height = height).
#        camInfo.roi

        return camInfo

    def createBagOfRGBDImage(self,sensorName,readings):
        bagName = self.datasetName+'_'+self.roomName+'_'+sensorName
        print bagName

        if not os.path.exists(self.dir+self.datasetName+'/'+self.roomName+'/'):
            os.makedirs(self.dir+self.datasetName+'/'+self.roomName+'/')

        bagRGB   =rosbag.Bag(self.dir+self.datasetName+'/'+self.roomName+'/'+self.datasetName+'_'+self.roomName+'_RGB_'+sensorName[5]+'.bag', 'w')
        bagDepth =rosbag.Bag(self.dir+self.datasetName+'/'+self.roomName+'/'+self.datasetName+'_'+self.roomName+'_D_'+sensorName[5]+'.bag', 'w')

        camInfo = self.getCameraInfo()

        try:
            for r in readings:
                if r.label == sensorName:
                    print("Adding %i" % r.id)

                    camInfo.header.seq = r.id
                    camInfo.header.stamp = rospy.Time(r.time_stamp)
                    camInfo.header.frame_id = "camera/"+r.label

                    print("RGB data")

#                    im_rgb = r.read_image_PIL(self.fulldir+str(r.id)+"_intensity.png")
                    im_rgb = r.read_image_CV(self.fulldir+str(r.id)+"_intensity.png")

                    im_rgb.header.seq = r.id
                    im_rgb.header.stamp = rospy.Time(r.time_stamp)
                    im_rgb.header.frame_id = "camera/"+r.label

                    im_rgb.encoding = "rgb8"  # Encoding of pixels -- channel meaning, ordering, size
                                              # taken from the list of strings in include/sensor_msgs/image_encodings.h

#                    bag.write('camera/'+r.label+'/intensity', im_rgb, im_rgb.header.stamp)
                    bagRGB.write('camera/'+r.label+'/rgb/image_raw', im_rgb, im_rgb.header.stamp)
                    bagRGB.write('camera/'+r.label+'/rgb/camera_info', camInfo, camInfo.header.stamp)

                    print("DEPTH data")

                    im_depth = r.read_image_CV(self.fulldir+str(r.id)+"_depth.png")

                    im_depth.header.seq = r.id
                    im_depth.header.stamp = rospy.Time(r.time_stamp)
                    im_depth.header.frame_id = "camera/"+r.label

                    im_depth.encoding = "mono16"  # Encoding of pixels -- channel meaning, ordering, size
                                                 # taken from the list of strings in include/sensor_msgs/image_encodings.h

#                    bag.write('camera/'+r.label+'/depth', im_depth, im_depth.header.stamp)
                    bagDepth.write('camera/'+r.label+'/depth/image_raw', im_depth, im_depth.header.stamp)
                    bagDepth.write('camera/'+r.label+'/depth/camera_info', camInfo, camInfo.header.stamp)

        finally:
            bagRGB.close()
            bagDepth.close()




    def processLaserScans(self):
        print "Processing laser scans ..."

        for filename in os.listdir(self.fulldir):
            if filename.endswith(".txt"):
                sensorName = os.path.splitext(filename)[0]

                readings = []

                print sensorName, " -- Loading data."
                for line in open(self.fulldir+sensorName+".txt","r"):
                    arr = line.split()
                    if(arr[0]!='#'):
                        l = MyLaserScan(arr)
                        l.readScan(self.fulldir+sensorName+'/'+str(l.id)+'_scan.txt')
                        readings.append(l)

                for r in readings:
                    print r

                self.createBagOfLaserScans(sensorName,readings)

                print '\n\n'

    def processRGBDdata(self):
        print "Processing RGBD data ..."

        cameras = dict()
        readings = []

        for line in open(os.path.normpath(self.fulldir)+".txt","r"):
            arr = line.split()
            if(arr[0]!='#'):
                im = MyRGBD(arr)

                if im.label not in cameras:
                    cameras[im.label] = 0
                cameras[im.label] += 1

                readings.append(im)

        for c in cameras:
            for r in readings:
                if r.label == c:
                    print r

        for c in cameras:
            print c,cameras[c]
            self.createBagOfRGBDImage(c,readings)

#        self.createBagsOfImages(readings)


    def process(self):
        if(self.type == 'LASER'):
            self.processLaserScans()
        elif(self.type == 'RGBD'):
            self.processRGBDdata()

if __name__ == "__main__":

    if len( sys.argv ) < 2:
        print('Usage: bagscans datasetdir')
    else:
        input = Input(sys.argv[1:])
        input.printAll()
        input.process()



