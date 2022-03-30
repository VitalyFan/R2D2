#imports
import math
import rospy
import cv2
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

qr = '2.5 2.5'

#publish topics
oil_pub = rospy.Publisher('/oil_detect', Image) 
defect_pub = rospy.Publisher('/defect_detect', Image)
#LINE_pub = rospy.Publisher('/line_detect', Image) 

#qr recognition
def image_qr_callback(data):
    global qr
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    barcodes = pyzbar.decode(cv_image)
    for barcode in barcodes:
        qr = barcode.data.decode("utf-8")

#line follow
def image_line_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    img_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    yellow = cv2.inRange(img_hsv, (23,134,186), (90,180,220))

    #contours, hierarchy = cv2.findContours( yellow.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #cv2.drawContours(cv_image, contours, -1, (255,255,0), 3, cv2.LINE_AA, hierarchy, 1) 
    #LINE_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))

    contours_yll , _ = cv2.findContours(yellow.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_yll = list(contours_yll)
    contours_yll.sort(key=cv2.minAreaRect)

    if len(contours_yll) > 0:
        cnt = contours_yll[0]
        if cv2.contourArea(cnt) > 150:
            rect = cv2.minAreaRect(cnt)
            (x_min, y_min), (w_min, h_min), angle = rect
            if angle < -45:
                angle = 90 + angle
            if w_min < h_min and angle > 0:
                angle = (90 - angle) * -1
            if w_min > h_min and angle < 0:
                angle = 90 + angle


            center = cv_image.shape[1] / 2
            error = x_min - center 

            set_velocity(vx=0.25, vy=error*(-0.01), vz=0, yaw=float('nan'), yaw_rate=angle*(0.008), frame_id='body')

#publish topics
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

    black = cv2.inRange(img_hsv, (0, 0, 0), (51,255,55)) 
    
    contours, hierarchy = cv2.findContours( black.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    cv2.drawContours(cv_image, contours, -1, (180, 105, 255), 3, cv2.LINE_AA, hierarchy, 1) 
    defect_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))

#subscribers
image_sub_qr = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_qr_callback, queue_size=1)
image_sub_publish_oil = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_publish_oil_callback, queue_size=1)
image_sub_publish_defect = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_publish_defect_callback, queue_size=1)
image_sub_line = rospy.Subscriber('main_camera/image_raw', Image, image_line_callback, queue_size=1)

#flight
navigate_wait(z=0.5, frame_id='body', auto_arm=True)
rospy.sleep(2)
telemetry = get_telemetry(frame_id='aruco_map')
xl = telemetry.x
yl = telemetry.y
print(qr)
qr = qr.split()
xt = float(qr[0])
yt = float(qr[1])
navigate_wait(x=xt, y=yt)

# rospy.sleep(3)
# navigate_wait(x = xl, y = yl)
# land_wait()
