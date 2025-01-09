import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image

bridge = CvBridge()

class Resize:
    def __init__(self):

        rospy.init_node('resize_image_node')
        self.resize_image_pub = rospy.Publisher("/usb_cam/image_resized", Image, queue_size=10)
        self.col = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)

        self.bridge = CvBridge()

    def callback(self, msg):
        try:
            # 從 ROS 訂閱影像轉換為 OpenCV 格式
            high_res_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 調降解析度（例如縮小到 640x480）
            low_res_image = cv2.resize(high_res_image, (640, 480), interpolation=cv2.INTER_NEAREST)
            
            # 將影像轉換回 ROS 格式
            low_res_image_msg = self.bridge.cv2_to_imgmsg(low_res_image, "bgr8")
            
            # 更新影像時間戳
            low_res_image_msg.header.stamp = rospy.Time.now()

            # publish low res img
            self.resize_image_pub.publish(low_res_image_msg)

        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

# # ROS init node
# rospy.init_node('resize_image_node')
# rospy.Subscriber("/usb_cam/image_raw", Image, callback)
# publisher = rospy.Publisher("/usb_cam/image_resized", Image, queue_size=10)

# keep node run
if __name__ == '__main__':
    try:
        node = Resize()
        rospy.spin()
    except:
        pass