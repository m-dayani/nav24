#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import numpy as np
import sys

rospy.init_node("yolo_node", anonymous=True)
proj_path = rospy.get_param('~path_proj', '../..')
rospy.loginfo('project path: %s', proj_path)
sys.path.append(os.path.join(proj_path, 'scripts'))
from aun_dl_special import TrackerYOLO


class YoloNode(TrackerYOLO):
    def __init__(self, model):
        super().__init__(model)
        # Params
        self.image = None
        self.lastTs = -1.0
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        # self.loop_rate = rospy.Rate(1)

        # Publishers
        self.pub = rospy.Publisher('/obj/coords', String, queue_size=10)

        # Subscribers
        rospy.Subscriber("/camera/image", Image, self.callback)

    def callback(self, msg):
        # rospy.loginfo('Image received...')
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.br.imgmsg_to_cv2(msg, "bgr8")
            ts = msg.header.stamp.to_nsec()
        except CvBridgeError as e:
            print(e)
        else:
            self.image = cv2_img
            self.lastTs = ts

    def get_string(self, ts, bbox):
        bbox_str = f'%ld' % ts
        for coord in bbox:
            bbox_str += f' %.4f' % coord
        # print(bbox_str)
        return bbox_str

    def start(self):
        # rospy.spin()
        while not rospy.is_shutdown():
            # rospy.loginfo('publishing image')
            # br = CvBridge()
            if self.image is not None:
                # rospy.loginfo('processing image')
                img = np.copy(self.image)
                ts = self.lastTs
                if not self.initialized:
                    ret, bbox = self.init(img, [])
                else:
                    ret, bbox = self.update(img)

                if ret:
                    res = self.get_string(ts, self.get_bbox_cv())
                    # print(res)
                    self.pub.publish(res)

            # self.loop_rate.sleep()


if __name__ == '__main__':
    # data_dir = os.getenv('DATA_PATH')
    data_dir = rospy.get_param('~path_ds', '.')
    model_path = os.path.join(data_dir, 'obj_tr_cap', 'models', 'm-omoumi-cap.pt')
    my_node = YoloNode(model_path)
    my_node.start()
