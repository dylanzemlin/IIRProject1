#!/usr/bin/env python

import sys
import threading
import math

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from rosgraph_msgs.msg import Log

from PyQt5.QtCore import Qt, QTimer, QPointF
from PyQt5.QtGui import QPixmap, QImage, qRgb, QPen, QColor, QPolygonF
from PyQt5.QtWidgets import (
    QApplication, QWidget, QTabWidget, QListWidget, QListWidgetItem,
    QLabel, QVBoxLayout, QFormLayout, QMainWindow,
    QLineEdit, QTextEdit, QPushButton, QTableWidget,
    QTableWidgetItem, QHeaderView, QGraphicsScene, QGraphicsView,
    QGraphicsPolygonItem, QGraphicsLineItem
)



class ZoomPanGraphicsView(QGraphicsView):
    def __init__(self, scene):
        QGraphicsView.__init__(self, scene)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.zoom_factor = 1.15

    def wheelEvent(self, event):
        if event.angleDelta().y() > 0:
            self.scale(self.zoom_factor, self.zoom_factor)
        else:
            self.scale(1.0 / self.zoom_factor, 1.0 / self.zoom_factor)



class MainWindow(QMainWindow):

    def __init__(self):
        QMainWindow.__init__(self)

        self.setWindowTitle("ITIR Tour Guide Robot | Fall 2025")

        # Robot pose
        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = 0.0

        # Map and path
        self.latest_map_msg = None
        self.global_path = []
        self.map_dirty = False

        # Logs
        self.last_logs = []
        self.logs_dirty = False

        # Speed
        self.current_speed_val = 0.0

        # Waypoint tracking for execution tab
        self.current_goal_str = "N/A"
        self.prev_goal_str = "N/A"

        self.tabs = QTabWidget()
        self.tabs.addTab(self.build_planning_tab(), "Planning")
        self.tabs.addTab(self.build_execution_tab(), "Execution")
        self.tabs.addTab(self.build_logging_tab(), "Logs")
        self.tabs.addTab(self.build_map_tab(), "Simulation Map")
        self.tabs.addTab(self.build_references_tab(), "References")
        self.setCentralWidget(self.tabs)

        self.pub_tour = rospy.Publisher("/tour_start", String, queue_size=1)

        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)
        rospy.Subscriber("/rosout_agg", Log, self.log_callback)
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self.path_callback)
        rospy.Subscriber("/move_base/NavfnROS/plan", Path, self.path_callback)

    def build_planning_tab(self):
        tab = QWidget()
        layout = QVBoxLayout()

        label = QLabel("Rearrange the waypoints:")
        layout.addWidget(label)

        self.waypoints = QListWidget()
        self.waypoints.setDragDropMode(QListWidget.InternalMove)
        for wp in ["REPF", "Felgar", "Devon", "Carson", "Sarkeys", "The Union"]:
            self.waypoints.addItem(QListWidgetItem(wp))
        layout.addWidget(self.waypoints)

        btn = QPushButton("Start Tour")
        btn.clicked.connect(self.send_tour_request)
        layout.addWidget(btn)

        tab.setLayout(layout)
        return tab

    def build_execution_tab(self):
        tab = QWidget()
        layout = QVBoxLayout()

        form = QFormLayout()

        self.current_speed = QLineEdit()
        self.prev_waypoint = QLineEdit()
        self.next_waypoint = QLineEdit()
        self.time_to_next = QLineEdit()
        self.time_to_completion = QLineEdit()

        for box in [
            self.current_speed,
            self.prev_waypoint,
            self.next_waypoint,
            self.time_to_next,
            self.time_to_completion,
        ]:
            box.setReadOnly(True)

        form.addRow("Current Speed:", self.current_speed)
        form.addRow("Previous Waypoint:", self.prev_waypoint)
        form.addRow("Next Waypoint:", self.next_waypoint)
        form.addRow("Time to Next Segment:", self.time_to_next)
        form.addRow("Total Time Remaining:", self.time_to_completion)

        layout.addLayout(form)
        tab.setLayout(layout)
        return tab

    def build_logging_tab(self):
        tab = QWidget()
        layout = QVBoxLayout()

        self.log_table = QTableWidget()
        self.log_table.setColumnCount(3)
        self.log_table.setHorizontalHeaderLabels(["Timestamp", "Level", "Message"])
        self.log_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

        layout.addWidget(self.log_table)
        tab.setLayout(layout)
        return tab

    def build_map_tab(self):
        tab = QWidget()
        layout = QVBoxLayout()

        label = QLabel("Top-down Map (from /map)")
        layout.addWidget(label)

        self.map_scene = QGraphicsScene()
        self.map_view = ZoomPanGraphicsView(self.map_scene)
        layout.addWidget(self.map_view)

        tab.setLayout(layout)
        return tab

    def build_references_tab(self):
        tab = QWidget()
        layout = QVBoxLayout()

        authors = QTextEdit()
        authors.setReadOnly(True)
        authors.setText("Dylan Zemlin\nSpencer Smith\nKenneth Thompson\nAlexander Greus")
        layout.addWidget(authors)

        tab.setLayout(layout)
        return tab

    def send_tour_request(self):
        pts = []
        for i in range(self.waypoints.count()):
            pts.append(str(self.waypoints.item(i).text()))
        msg = ",".join(pts)
        rospy.loginfo("UI sending tour: %s" % msg)
        self.pub_tour.publish(msg)

    def odom_callback(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed_val = math.sqrt(vx * vx + vy * vy)

        if self.robot_x is None:
            self.robot_x = msg.pose.pose.position.x
            self.robot_y = msg.pose.pose.position.y

        if self.latest_map_msg is not None:
            self.map_dirty = True

    def amcl_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

        if self.latest_map_msg is not None:
            self.map_dirty = True

    def path_callback(self, msg):
        if self.global_path:
            last = self.global_path[-1]
            self.prev_goal_str = "(%.2f, %.2f)" % (last[0], last[1])

        self.global_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        rospy.loginfo("UI: Received path with %d points" % len(self.global_path))

        if self.global_path:
            goal = self.global_path[-1]
            self.current_goal_str = "(%.2f, %.2f)" % (goal[0], goal[1])

        if self.latest_map_msg is not None:
            self.map_dirty = True

    def map_callback(self, msg):
        self.latest_map_msg = msg
        self.map_dirty = True

    def log_callback(self, msg):
        try:
            ts = msg.header.stamp.to_sec()
        except:
            ts = rospy.get_time()
        lvl = getattr(msg, "level", -1)
        txt = getattr(msg, "msg", str(msg))

        self.last_logs.append((ts, lvl, txt))
        if len(self.last_logs) > 60:
            self.last_logs.pop(0)

        self.logs_dirty = True

    def update_log_table(self):
        if not hasattr(self, "log_table"):
            return

        self.log_table.setRowCount(len(self.last_logs))
        for i, entry in enumerate(self.last_logs):
            ts, lvl, txt = entry
            self.log_table.setItem(i, 0, QTableWidgetItem("%.2f" % ts))
            self.log_table.setItem(i, 1, QTableWidgetItem(str(lvl)))
            self.log_table.setItem(i, 2, QTableWidgetItem(txt))
        self.log_table.scrollToBottom()

    def compute_time_estimates(self):
        if not self.global_path or self.robot_x is None:
            return "N/A", "N/A"

        speed = self.current_speed_val
        if speed < 0.05:
            speed = 0.05

        # Distance from robot to first path point
        p0x, p0y = self.global_path[0]
        curr_dist = math.hypot(p0x - self.robot_x, p0y - self.robot_y)

        # Total distance along path
        total_dist = curr_dist
        for i in range(len(self.global_path) - 1):
            x1, y1 = self.global_path[i]
            x2, y2 = self.global_path[i + 1]
            total_dist += math.hypot(x2 - x1, y2 - y1)

        t_curr = curr_dist / speed
        t_total = total_dist / speed

        return "%.1f sec" % t_curr, "%.1f sec" % t_total

    def draw_robot(self):
        if self.robot_x is None or self.latest_map_msg is None:
            return

        # Make the robot clearly visible
        size = 1.0

        p1 = QPointF(0.0, size)
        p2 = QPointF(-size / 2.0, -size / 2.0)
        p3 = QPointF(size / 2.0, -size / 2.0)
        triangle = QPolygonF([p1, p2, p3])

        item = QGraphicsPolygonItem(triangle)
        item.setBrush(QColor(255, 0, 0))
        item.setPen(QPen(Qt.black, 0))

        res = self.latest_map_msg.info.resolution
        ox = self.latest_map_msg.info.origin.position.x
        oy = self.latest_map_msg.info.origin.position.y
        height = self.latest_map_msg.info.height

        px = (self.robot_x - ox) / res
        py = height - ((self.robot_y - oy) / res)

        item.setPos(px, py)
        item.setRotation(-math.degrees(self.robot_yaw))

        self.map_scene.addItem(item)
        self.map_view.centerOn(px, py)

    def draw_path(self):
        if self.latest_map_msg is None:
            return
        if len(self.global_path) < 2:
            return

        res = self.latest_map_msg.info.resolution
        ox = self.latest_map_msg.info.origin.position.x
        oy = self.latest_map_msg.info.origin.position.y
        height = self.latest_map_msg.info.height

        if self.robot_x is not None:
            rx = self.robot_x
            ry = self.robot_y
            tx = self.global_path[0][0]
            ty = self.global_path[0][1]

            rpx = (rx - ox) / res
            rpy = height - ((ry - oy) / res)
            tpx = (tx - ox) / res
            tpy = height - ((ty - oy) / res)

            seg = QGraphicsLineItem(rpx, rpy, tpx, tpy)
            seg.setPen(QPen(QColor(255, 0, 0), 2))
            self.map_scene.addItem(seg)

        pen = QPen(QColor(0, 0, 255), 1)
        for i in range(len(self.global_path) - 1):
            x1, y1 = self.global_path[i]
            x2, y2 = self.global_path[i + 1]

            px1 = (x1 - ox) / res
            py1 = height - ((y1 - oy) / res)
            px2 = (x2 - ox) / res
            py2 = height - ((y2 - oy) / res)

            line = QGraphicsLineItem(px1, py1, px2, py2)
            line.setPen(pen)
            self.map_scene.addItem(line)

        self.prev_waypoint.setText(self.prev_goal_str)
        self.next_waypoint.setText(self.current_goal_str)

        t_curr, t_total = self.compute_time_estimates()
        self.time_to_next.setText(t_curr)
        self.time_to_completion.setText(t_total)

    def update_map_view(self):
        msg = self.latest_map_msg
        if msg is None:
            return

        w = msg.info.width
        h = msg.info.height
        data = msg.data

        if w == 0 or h == 0:
            return

        img = QImage(w, h, QImage.Format_RGB888)
        for y in range(h):
            for x in range(w):
                val = data[x + y * w]
                if val == 0:
                    c = qRgb(255, 255, 255)
                elif val == 100:
                    c = qRgb(0, 0, 0)
                else:
                    c = qRgb(127, 127, 127)
                img.setPixel(x, h - 1 - y, c)

        pix = QPixmap.fromImage(img)

        self.map_scene.clear()
        self.map_scene.addPixmap(pix)

        self.draw_path()
        self.draw_robot()

        self.map_scene.setSceneRect(0, 0, w, h)

    def on_timer(self):
        self.current_speed.setText("%.2f m/s" % self.current_speed_val)

        if self.logs_dirty:
            self.update_log_table()
            self.logs_dirty = False

        if self.map_dirty:
            self.update_map_view()
            self.map_dirty = False


def ros_spin_thread():
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node("tour_guide_ui", anonymous=False)

    ros_thread = threading.Thread(target=ros_spin_thread)
    ros_thread.daemon = True
    ros_thread.start()

    app = QApplication(sys.argv)
    window = MainWindow()
    window.resize(950, 750)
    window.show()

    timer = QTimer()
    timer.timeout.connect(window.on_timer)
    timer.start(100)

    app.exec_()

    rospy.signal_shutdown("GUI closed")
