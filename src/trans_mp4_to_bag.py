import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import subprocess
import time
import os

def mp4_to_bag(video_path, topic_name="/usb_cam/image_raw", bag_filename="online_video_0.25.bag"):
    # 檢查影片檔案是否存在
    if not os.path.exists(video_path):
        print(f"File not found: {video_path}")
        return
    
    time.sleep(5)  # 等待 rosbag record 完全啟動
    rosbag_proc = subprocess.Popen(["rosbag", "record", "-O", bag_filename, topic_name])

    cap = cv2.VideoCapture(video_path)

    # 新增檢查影片是否開啟成功
    if not cap.isOpened():
        rospy.logerr("Unable to open video file: " + video_path)
        rosbag_proc.terminate()  # 停止 rosbag record
        rosbag_proc.wait()       # 等待進程結束
        return
    
    rospy.init_node('video_publisher', anonymous=True)
    pub = rospy.Publisher(topic_name, Image, queue_size=10)
    bridge = CvBridge()
    rate = rospy.Rate(30)  # 根據影片的 FPS 設置
    
    time.sleep(5)  # 等待 rosbag record 啟動

    try:
        while not rospy.is_shutdown() and cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            pub.publish(msg)
            rate.sleep()
    finally:
        cap.release()
        # 結束 rosbag record
        rosbag_proc.terminate()
        rosbag_proc.wait()

if __name__ == "__main__":
    video_path = "Screencast 2024-11-21 02:35:14.mp4"  # 替換為您的 MP4 路徑
    mp4_to_bag(video_path)
