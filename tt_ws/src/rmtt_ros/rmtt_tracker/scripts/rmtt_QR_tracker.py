#!/usr/bin/env python3

import rospy
import rospkg
import sensor_msgs.msg
#import std_msgs.msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import pyzbar.pyzbar as pyzbar

# resize the image to w*h
fw = 360
fh = 240
pid_w=[0.8,0,0]#yaw pian hang
pid_h=[0.8,0,0]#up chui zhi
pid_f=[0.5,0,0]# qian hou
pid_l=[1.0,0,0]# shui ping
#zero_twist_published=False
def callback(msg):
    print("enter callback")
    global zero_twist_published
    

    img = bridge.imgmsg_to_cv2(msg)
    img = cv2.resize(img, (fw, fh))   
    #img , info = findFace(img)
    gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    zero_twist_published=False
    barcodes=pyzbar.decode(gray)
    if barcodes:
        print("enter barcodes")
        barcode=barcodes[0]
        points=[]
        for point in barcode.polygon:
            points.append([point[0],point[1]])
        points=np.array(points,dtype=np.int32).reshape(-1,1,2)
        cv2.polylines(img,[points],isClosed=True,color=(0,0,255),thickness=2)
        #
        x1=points[0,0,0]
        y1=points[0,0,1]
        x2=points[1,0,0]
        y2=points[1,0,1]
        x3=points[2,0,0]
        y3=points[2,0,1]
        x4=points[3,0,0]
        y4=points[3,0,1]
        hl=np.sqrt(np.square((x1-x4))+np.square((y1-y4)))
        hr=np.sqrt(np.square((x2-x3))+np.square((y2-y3)))
        ratio=hl/hr*100

        (x,y,w,h)=barcode.rect
        cv2.rectangle(img,(x,y),(x+w,y+h),(225,225,225),2)

        barcodeData=barcode.data.decode("utf-8")
        barcodeType=barcode.type

        text="{}({})".format(barcodeData,barcodeType)
        cv2.putText(img,text,(x,y-10),cv2.FONT_HERSHEY_SIMPLEX,
        .5,(225,225,225),2)
        print("[INFO]x:{} y:{} h:{}  ratio:{} Found {} barcode:".format(x,y,w,h,ratio,barcodeType,barcodeData))
        cx=x+w//2
        cy=y+h//2
        area=w*h
    else:
        print("enter barcodes else")
        cx=0
        cy=0
        area=0
        ratio=0
    cv2.imshow("qrcode_detect_result",img)
    cv2.waitKey(1)
    yaw_speed,up_speed,forward_speed,left_speed =pidtrack(cx,cy,area,ratio,fw,fh,pid_w,pid_h,pid_f,pid_l)
    rc="forward:"+str(forward_speed)+"up:"+str(up_speed)+"yaw:"+str(yaw_speed)+"left:"+str(left_speed)
    speed=Twist()

    #yaw_speed , forward_speed=trackFace(info,w,h,pid_w,pid_f)
    #rc="left:"+"0"+"forward:"+str(forward_speed)+"up:"+str(up_speed)+"yaw:"+str(yaw_speed)
    #speed=Twist()
    if yaw_speed !=0 or forward_speed !=0 or up_speed !=0 or left_speed!=0:
        speed.linear.x=-forward_speed
        speed.linear.y=-left_speed
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
            rospy.loginfo("no object detected")

    #cv2.imshow('Frame', img)
    #cv2.waitKey(1)

def pidtrack(cx,cy,area,ratio,fw,fh,pid_w,pid_h,pid_f,pid_l):
    error_w=cx-fw//2
    speed_w=pid_w[0]*error_w
    speed_w=int(np.clip(speed_w,-100,100))/100.0

    error_h=cy-fh//2
    speed_h=pid_h[0]*error_h
    speed_h=int(np.clip(speed_h,-100,100))/100.0

    error_f=np.sqrt(area)-70
    speed_f=pid_f[0]*error_f
    speed_f=int(np.clip(speed_f,-100,100))/100.0

    error_l=ratio-100
    speed_l=pid_l[0]*error_l
    speed_l=int(np.clip(speed_l,-100,100))/100.0

    if cx !=0:
        yaw_speed=speed_w
    else:
        yaw_speed=0
    if cy !=0:
        up_speed=speed_h
    else:
        up_speed=0
    if area !=0:
        forward_speed=speed_f
    else:
        forward_speed=0
    if ratio !=0:
        left_speed= speed_l
    else:
        left_speed=0

    return yaw_speed,up_speed,forward_speed,left_speed


if __name__ == '__main__':

    #rp = rospkg.RosPack()
    #path = rp.get_path("rmtt_tracker")
    #faceCascade=cv2.CascadeClassifier(path + '/config/haarcascade_frontalface_default.xml')
    bridge = CvBridge()
    rospy.init_node('qrcode_tracker')
    print("main function has launched")
    rospy.Subscriber("image_raw", Image, callback)
    print("main function has launched1")
    pub=rospy.Publisher('cmd_vel',Twist,queue_size=1)
    print("main function has launched2")
    rospy.spin()

""" def findFace(img):
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
            return img , [[0,0],0] """

""" def trackFace(info,w,h,pid_w,pid_f):
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

    return yaw_speed , forward_speed , up_speed """



