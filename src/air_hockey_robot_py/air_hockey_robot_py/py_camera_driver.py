
#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np

class PyCameraDriver(Node):

    def __init__(self):
        super().__init__('py_camera_driver')
        self.image_publisher = self.create_publisher(Image, 'undistorted_image', 10)
        self.image_subscriber = self.create_subscription(Image, 'usb_camera_image', self.callback, 10)
        # self.mtx = np.array([[349.6116864,    0.0,         327.94408941], [  0.0,         473.27671685, 243.14884448], [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
        # self.dist = np.array([[-0.36112722,  0.16562489, -0.00724717, -0.00414701, -0.04018561]])
        # self.timer = self.create_wall_timer(1.0 / 120.0, self.callback)
        # self.timer = self.create_timer(1.0 / 1000.0, self.callback)
        self.bridge = CvBridge()
        # termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((7*10,3), np.float32)
        objp[:,:2] = np.mgrid[0:10,0:7].T.reshape(-1,2)
        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.
        for n in range(0, 100, 3):
            img = cv2.imread('/home/osx/Downloads/fig/camera_capture{}.jpg'.format(n))
            img = cv2.resize(img,(640, 480))
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, (10, 7), None)
            if ret == True:
                objpoints.append(objp)
                corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
                imgpoints.append(corners2)
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        print(ret)
        print(mtx)
        print(dist)
        h, w = 480, 640
        self.mtx = mtx
        self.dist = dist
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w, h), 1, (w, h))
        self.newcameramtx = newcameramtx
        self.roi = roi
        
    def callback(self, msg):
        # ret = False
        # while not ret:
        #     ret, frame = self.cap.read()
        # if ret:
        frame = self.bridge.imgmsg_to_cv2(msg)
        # print(frame.shape[:2])
        # h,  w = frame.shape[:2]
        # print(self.mtx, self.dist, (w, h), 1, (w, h))
        dst = cv2.undistort(frame, self.mtx, self.dist, None, self.newcameramtx)
        x,y,w,h = self.roi
        dst = dst[y:y+h, x:x+w]
        # dst = cv2.resize(frame, (100, 100))
        image_message = self.bridge.cv2_to_imgmsg(dst, encoding="passthrough")
            # image_message = self.bridge.cv2_to_imgmsg(frame, encoding="passthrough")
        self.image_publisher.publish(image_message)
        return True
        # else:
        #     return False

def main(args=None):
    rclpy.init(args=args)
    camera_driver = PyCameraDriver()
    rclpy.spin(camera_driver)
    camera_driver.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
