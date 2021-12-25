#!/usr/bin/env python3

import rospy
import rospkg
import std_msgs.msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

# resize the image to w*h
w = 360
h = 240
pid_w=[0.5,0,0]
pid_f=[0.8,0,0]
zero_twist_published=False
def callback(msg):
    global zero_twist_published

    img = bridge.imgmsg_to_cv2(msg)
    img = cv2.resize(img, (w, h))   
    img , info = findFace(img)
    yaw_speed , forward_speed=trackFace(info,w,h,pid_w,pid_f)
    rc="left:"+"0"+"forward:"+str(forward_speed)+"up:"+str(up_speed)+"yaw:"+str(yaw_speed)
    speed=Twist()
    if yaw_speed !=0 or forward_speed !=0 or up_speed !=0:
        speed.linear.x=-forward_speed
        speed.linear.y=0.0
        speed.linear.z=-up_speed
        speed.angular.x=0.0

        speed.angular.y=0.0
        speed.angular.z=-yaw_speed
        rospy.loginfo(rc)
        pub.publish(speed)
        zero_twist_published=False
    else:
        if not zero_twist_published:
            pub.publish(speed)
            zero_twist_published=True
            rospy.loginfo("no face detected")

    cv2.imshow('Frame', img)
    cv2.waitKey(1)

def findFace(img):
    img_gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    faces=faceCascade.detectMultiScale(img_gray,1.1,6)
    myFaceListC=[]
    myFaceListArea=[]
    for (x,y,w,h) in faces:
        cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,225),2)
        cx=x+w
        cy=y+h
        area=w*h
        myFaceListArea.append(area)
        myFaceListC.append([cx,cy])
        if len(myFaceListArea) !=0:
            i=myFaceListArea.index(max(myFaceListArea))
            return img , [myFaceListC[i],myFaceListArea[i]]
        else:
            return img , [[0,0],0]

def trackFace(info,w,h,pid_w,pid_f):
    error_w=info[0][0]-w
    speed_w=pid_w[0]*error_w
    speed_w=int(np.clip(speed_w,-100,100))

    error_f=np.sqrt(info[1])-70
    speed_f=pid_f[0]*error_f
    speed_f=int(np.clip(speed_f,-100,100))

    if info[0][0]!=0:
        yaw_speed=speed_w
    else:
        yaw_speed=0

    if info[0][0]!=0:
        forward_speed=speed_w
    else:
        forward_speed=0

    return yaw_speed , forward_speed , up_speed


if __name__ == '__main__':

    rp = rospkg.RosPack()
    path = rp.get_path("rmtt_tracker")
    faceCascade=cv2.CascadeClassifier(path + '/config/haarcascade_frontalface_default.xml')
    bridge = CvBridge()
    rospy.init_node('face_tracker', anonymous=True)
    
    sub = rospy.Subscriber("image_raw", Image, callback)
    pub=rospy.Publisher('cmd_vel',Twist,queue_size=1)
    rospy.spin()
