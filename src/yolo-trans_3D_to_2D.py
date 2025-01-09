import rospy
from geometry_msgs.msg import PointStamped, Quaternion
from yolo.msg import LabeledPointArray  # 假設這個訊息包含 x, y, z, labels
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

class CoordinateTransformerNode:
    def __init__(self):
        rospy.init_node("coordinate_transformer")
        
        # 訂閱來自 YOLO 節點的三維座標及顏色標籤
        self.world_point_sub = rospy.Subscriber("/yolo/objects/world_point_array", LabeledPointArray, self.callback)

        # 發佈到 RViz 的 2D 投影點
        self.marker_pub = rospy.Publisher("/yolo/objects/visualization_marker_array", MarkerArray, queue_size=10)

        # 發佈相對於車輛初始位置的 2D 座標
        self.coord_pub = rospy.Publisher("/yolo/objects/relative_coordinates", LabeledPointArray, queue_size=10)

        # 車子的初始位置
        self.initial_x = 0
        self.initial_y = 0
        self.initial_position_set = True

    def create_identity_quaternion(self):
        """
        創建一個單位四元數，表示無旋轉。
        """
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = 0.0
        q.w = 1.0  # 這是單位四元數，代表無旋轉
        return q

    def callback(self, msg):
        points_3d = np.array(list(zip(msg.x, msg.y, msg.z)))
        labels = msg.labels  # 取得 YOLO 的標籤（假設這是正確的欄位名）

        # 如果尚未設定初始位置，將第一個點作為初始位置
        if not self.initial_position_set:
            self.initial_x = points_3d[0][0]
            self.initial_y = points_3d[0][1]
            self.initial_position_set = True

        # 將座標轉換為相對於初始位置的座標
        relative_points_2d = []
        relative_labels = []  # 儲存對應的顏色標籤
        for point, label in zip(points_3d, labels):
            relative_x = point[0] - self.initial_x
            relative_y = point[1] - self.initial_y
            distance = np.sqrt(relative_x**2 + relative_y**2)

            # 篩選距離小於等於 10 公尺的座標
            if distance <= 50.0:
                relative_points_2d.append([relative_x, relative_y])
                relative_labels.append(label)  # 保存相應的標籤

        relative_points_2d = np.array(relative_points_2d)

        # 正確初始化 MarkerArray 並使用 DELETEALL
        delete_marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.header.frame_id = "map"
        delete_marker_array.markers.append(delete_marker)

        # 發佈 MarkerArray 來刪除 Marker
        self.marker_pub.publish(delete_marker_array)
        #rospy.sleep(0.005)  # 等待 100 毫秒讓 RViz 更新

        # 建立新 MarkerArray
        marker_array = MarkerArray()
        for i, (x, y) in enumerate(relative_points_2d):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "projected_points"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0  # Z軸設為0
            marker.scale.x = 0.5  # 點的大小
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.a = 1.0  # 透明度

            # 根據標籤設置顏色
            if relative_labels[i] == "yellow_cone":
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0  # 黃色
            elif relative_labels[i] == "blue_cone":
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0  # 藍色
            elif relative_labels[i] == "orange_cone":
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0  # 紅色
            else:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0  # 預設為黑色

            # 設置無旋轉的單位四元數
            marker.pose.orientation = self.create_identity_quaternion()

            marker_array.markers.append(marker)

        # 發佈 MarkerArray 到 RViz
        self.marker_pub.publish(marker_array)

        # 發佈相對座標到新主題
        relative_coords_msg = LabeledPointArray()  # 根據 LabeledPointArray 的結構設置
        relative_coords_msg.x = [(p[0]) for p in relative_points_2d]
        relative_coords_msg.y = [(p[1]) for p in relative_points_2d]
        relative_coords_msg.z = [0.0] * len(relative_points_2d)  # z 坐標全為 0
        relative_coords_msg.labels = relative_labels  # 發佈標籤

        # 印出相對座標及標籤資訊
        print("Published Relative Coordinates:")
        for x, y, label in zip(relative_coords_msg.x, relative_coords_msg.y, relative_coords_msg.labels):
            print(f"x: {x}, y: {y}, label: {label}")

        # 發佈 2D 相對座標
        self.coord_pub.publish(relative_coords_msg)

if __name__ == '__main__':
    try:
        node = CoordinateTransformerNode()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
