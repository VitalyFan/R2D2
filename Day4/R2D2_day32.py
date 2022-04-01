#imports
import math
import rospy
import cv2
import numpy as np
from clover import srv
from std_srvs.srv import Trigger
from pyzbar import pyzbar
from cv_bridge import CvBridge
from sensor_msgs.msg import Image



#converting ROS image to OpenCV image
bridge = CvBridge()


#node inicialization
rospy.init_node('flight_and_detection')


#proxies
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)



#navigate_wait function
def navigate_wait(x=0, y=0, z=1.5, yaw=float('nan'), speed=0.7, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)


#landing function
def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)

#global data
qr = '2.5 2.5'
rev = 0
down_zero = 0
up_zero = 0
a = 0

#publish topics
camera_pub = rospy.Publisher('/camera', Image, queue_size = 10)
oil_pub = rospy.Publisher('/oil_detect', Image, queue_size = 10)
defect_pub = rospy.Publisher('/defect_detect', Image, queue_size = 10)


#qr recognition function
def image_qr_callback(data):
    global qr
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    barcodes = pyzbar.decode(cv_image)
    for barcode in barcodes:
        qr = barcode.data.decode("utf-8")


#line follow function
def image_line_callback(data):
    global rev, up_zero, down_zero, a
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    img_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(img_hsv, (25,120,120), (90,255,255))
    
    up = yellow[0:119, 0:319]
    down = yellow[119:239, 0:319]
    down_zero = np.count_nonzero(down)
    up_zero = np.count_nonzero(up)
    
    
    contours_yll , _ = cv2.findContours(yellow.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_yll = list(contours_yll)
    contours_yll.sort(key=cv2.minAreaRect)
    

    if len(contours_yll) > 0:
        cnt = contours_yll[0]
        
        a = cv2.contourArea(cnt)
        


# publish topics
def image_publish_oil_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    img_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    red = cv2.inRange(img_hsv, (2,40,150), (26,255,255))

    contours, hierarchy = cv2.findContours( red.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(cv_image, contours, -1, (255,0,0), 3, cv2.LINE_AA, hierarchy, 1)
    oil_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))

def image_publish_defect_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    img_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    black = cv2.inRange(img_hsv, (90, 150, 20), (105,255,45))

    contours, hierarchy = cv2.findContours( black.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(cv_image, contours, -1, (180, 105, 255), 3, cv2.LINE_AA, hierarchy, 1)
    defect_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))


# #subscribers
image_sub_qr = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_qr_callback, queue_size=1)
image_sub_publish_oil = rospy.Subscriber('main_camera/image_raw', Image, image_publish_oil_callback, queue_size=1)
image_sub_publish_defect = rospy.Subscriber('main_camera/image_raw', Image, image_publish_defect_callback, queue_size=1)



# #flight
navigate_wait(z=1, frame_id='body', auto_arm=True)
rospy.sleep(2)
telemetry = get_telemetry(frame_id='aruco_map')
xl = telemetry.x
yl = telemetry.y
#qr rocognistion
print(qr)
qr = qr.split()
#qr data processing
xt = float(qr[0])
yt = float(qr[1])

xo = float(qr[2])
yo = float(qr[3])

#navigation to lake zone
navigate_wait(x = xo, y = yo, z = 1)
rospy.sleep(2)

#water collection
navigate_wait(x = 0, y = 0, z = -0.5, frame_id='navigate_target')
rospy.sleep(6)
print('Successful water withdrawal')
navigate_wait(x = xo, y = yo, z = 1)

#start of monitoring
navigate_wait(x = xt, y = yt, z = 0.8)
rospy.sleep(4)
image_sub_line = rospy.Subscriber('main_camera/image_raw', Image, image_line_callback, queue_size=1)

if down_zero > up_zero:
    navigate_wait(x = 0, y = 0, z = 0, yaw=math.radians(-180), frame_id='body')
    rospy.sleep(3)
while a < 3500 and a > 1000:
    navigate_wait(x = 0.3, z =0, frame_id='navigate_target')

#navigation to landing zone and landing
navigate_wait(x = xl, y = yl)
land_wait()
#final report
print('Navigation area: x={}, y={}'.format(xt,yt))
print('Lake center: x={}, y={}'.format(xo,yo))
