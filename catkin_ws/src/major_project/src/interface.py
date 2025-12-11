#!/usr/bin/env python

import sys
import threading
import math
import numpy as np
import rospkg
import rospy

from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from rosgraph_msgs.msg import Log

from PyQt5.QtGui import QPainter, QPixmap, QImage, QPen, QColor
from PyQt5.QtWidgets import (
    QApplication, QWidget, QTabWidget, QListWidget, QListWidgetItem,
    QLabel, QVBoxLayout, QFormLayout, QMainWindow,
    QLineEdit, QTextEdit, QPushButton, QTableWidget,
    QTableWidgetItem, QHeaderView, QGraphicsScene, QGraphicsView,
    QGraphicsLineItem, QGraphicsEllipseItem, QGraphicsTextItem,
    QToolTip, QMessageBox, QGraphicsItem
)
from PyQt5.QtCore import Qt, QTimer


# Helper class for tooltip-enabled ellipses
class TooltipEllipseItem(QGraphicsEllipseItem):
    def __init__(self, *args, **kwargs):
        super(TooltipEllipseItem, self).__init__(*args, **kwargs)
        self.setAcceptHoverEvents(True)
        self.tooltip_text = ""

    def setTooltipText(self, text):
        self.tooltip_text = text

    def hoverEnterEvent(self, event):
        QToolTip.showText(event.screenPos(), self.tooltip_text)
        super(TooltipEllipseItem, self).hoverEnterEvent(event)

    def hoverLeaveEvent(self, event):
        QToolTip.hideText()
        super(TooltipEllipseItem, self).hoverLeaveEvent(event)


# Helper class for zooming and panning
class ZoomPanGraphicsView(QGraphicsView):
    def __init__(self, scene):
        super(ZoomPanGraphicsView, self).__init__(scene)

        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.zoom_factor = 1.15

        # fix tooltip issues
        self.setMouseTracking(True)
        self.viewport().setMouseTracking(True)
        self.setInteractive(True)
        self.setRenderHint(QPainter.Antialiasing)

        # improves hit registration during zooming
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorUnderMouse)
        self.setViewportUpdateMode(QGraphicsView.SmartViewportUpdate)

    def wheelEvent(self, event):
        if event.angleDelta().y() > 0:
            scale_factor = self.zoom_factor
        else:
            scale_factor = 1.0 / self.zoom_factor

        self.scale(scale_factor, scale_factor)
        event.accept()

    def mouseMoveEvent(self, event):
        super(ZoomPanGraphicsView, self).mouseMoveEvent(event)
        self.viewport().update()

    def enterEvent(self, event):
        super(ZoomPanGraphicsView, self).enterEvent(event)
        self.setMouseTracking(True)


class MainWindow(QMainWindow):

    def __init__(self):
        QMainWindow.__init__(self)

        self.setWindowTitle("ITIR Tour Guide Robot | Fall 2025")

        # Robot pose
        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = 0.0

        # Map / Path
        self.latest_map_msg = None
        self.global_path = []
        self.full_plan = []

        # Graphics items and flags
        self.map_pixmap_item = None
        self.robot_item = None
        self.path_lines = []
        self.waypoint_markers = []
        self.time_text_item = None

        self.map_image_dirty = False
        self.path_dirty = False
        self.robot_dirty = False
        self.plan_dirty = False

        # Current plan segment index
        self.current_seg_index = 0

        # Logs
        self.last_logs = []
        self.logs_dirty = False

        # Speed
        self.current_speed_val = 0.0

        # Execution view values
        self.current_goal_str = "N/A"
        self.prev_goal_str = "N/A"
        self.current_waypoint_name = "N/A"
        self.prev_waypoint_name = "N/A"

        # Tour time management
        self.tour_duration_sec = None
        self.tour_start_time = None
        self.timer_alert_shown = False

        # Robot freeze visualization
        self.robot_frozen = False
        self.freeze_text_item = None

        # Tabs
        self.tabs = QTabWidget()
        self.tabs.addTab(self.build_planning_tab(), "Planning")
        self.tabs.addTab(self.build_execution_tab(), "Execution")
        self.tabs.addTab(self.build_logging_tab(), "Logs")
        self.tabs.addTab(self.build_map_tab(), "Simulation Map")
        self.tabs.addTab(self.build_references_tab(), "References")
        self.setCentralWidget(self.tabs)

        # Tab indices
        self.LOG_TAB_INDEX = 2
        self.MAP_TAB_INDEX = 3

        self.pub_tour = rospy.Publisher("/tour_start", String, queue_size=1)

        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)
        rospy.Subscriber("/rosout_agg", Log, self.log_callback)
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self.path_callback)
        rospy.Subscriber("/move_base/NavfnROS/plan", Path, self.path_callback)
        rospy.Subscriber("/tour_plan_path", Path, self.tour_plan_callback)

    # loads in waypoints from the file
    def load_waypoint_file(self):
        rp = rospkg.RosPack()
        filepath = rp.get_path("major_project") + "/waypoints.tour"
        table = {}

        try:
            with open(filepath, "r") as f:
                for line in f:
                    line = line.strip()
                    if not line or ":" not in line:
                        continue

                    name, coords = line.split(":", 1)
                    table[name.strip()] = coords.strip()

            rospy.loginfo("UI: Loaded %d waypoints from file.", len(table))

        except Exception as e:
            rospy.logerr("UI: Failed to load waypoint file: %s", str(e))

        return table

    # builds a numeric table for waypoint coordinates
    def build_waypoint_coords(self):
        self.waypoint_coords = {}
        for name, coord_str in self.waypoint_table.items():
            try:
                parts = coord_str.split(",")
                if len(parts) < 2:
                    continue
                x = float(parts[0].strip())
                y = float(parts[1].strip())
                self.waypoint_coords[name] = (x, y)
            except Exception as e:
                rospy.logwarn(
                    "UI: Failed to parse coords for waypoint '%s': %s", name, str(e)
                )

    # finds the closest waypoint name to given coordinates
    def find_closest_waypoint_name(self, x, y):
        if not hasattr(self, "waypoint_coords") or not self.waypoint_coords:
            return "Unknown"

        best_name = "Unknown"
        best_dist = None
        for name, (wx, wy) in self.waypoint_coords.items():
            d = math.hypot(wx - x, wy - y)
            if best_dist is None or d < best_dist:
                best_dist = d
                best_name = name

        return best_name

    def build_planning_tab(self):
        tab = QWidget()
        layout = QVBoxLayout()

        label = QLabel("Rearrange the waypoints:")
        layout.addWidget(label)

        self.waypoints = QListWidget()
        self.waypoints.setDragDropMode(QListWidget.InternalMove)

        # Load from file
        self.waypoint_table = self.load_waypoint_file()
        self.build_waypoint_coords()

        for wp_name in self.waypoint_table.keys():
            self.waypoints.addItem(QListWidgetItem(wp_name))

        layout.addWidget(self.waypoints)

        # Time input
        layout.addWidget(QLabel("Desired Tour Duration (seconds):"))
        self.tour_duration_input = QLineEdit()
        self.tour_duration_input.setPlaceholderText("e.g., 600")
        layout.addWidget(self.tour_duration_input)

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
        self.time_remaining = QLineEdit()

        for box in [
            self.current_speed,
            self.prev_waypoint,
            self.next_waypoint,
            self.time_to_next,
            self.time_to_completion,
            self.time_remaining,
        ]:
            box.setReadOnly(True)

        form.addRow("Current Speed:", self.current_speed)
        form.addRow("Previous Waypoint:", self.prev_waypoint)
        form.addRow("Next Waypoint:", self.next_waypoint)
        form.addRow("Time to Next Segment:", self.time_to_next)
        form.addRow("Total Time Remaining (Path):", self.time_to_completion)
        form.addRow("Tour Time Remaining (Budget):", self.time_remaining)

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
        count = self.waypoints.count()

        # Highest priority for first element
        for i in range(count):
            name = str(self.waypoints.item(i).text())
            priority = count - i
            pts.append("%s:%d" % (name, priority))

        # Time budget
        duration_str = self.tour_duration_input.text().strip()
        if not duration_str.isdigit():
            rospy.logwarn("UI: Invalid or empty duration; using default of 600 sec")
            duration_str = "600"

        try:
            self.tour_duration_sec = int(duration_str)
        except ValueError:
            self.tour_duration_sec = 600

        self.tour_start_time = rospy.get_time()
        self.timer_alert_shown = False
        self.robot_frozen = False
        self.update_freeze_overlay()

        msg_str = "TIME=" + str(self.tour_duration_sec) + ";" + ",".join(pts)

        rospy.loginfo("UI sending tour: %s" % msg_str)
        self.pub_tour.publish(msg_str)

    def odom_callback(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed_val = math.sqrt(vx * vx + vy * vy)

        if self.robot_x is None:
            self.robot_x = msg.pose.pose.position.x
            self.robot_y = msg.pose.pose.position.y

        if self.latest_map_msg is not None:
            self.robot_dirty = True

    def amcl_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

        if self.latest_map_msg is not None:
            self.robot_dirty = True

    def path_callback(self, msg):
        if self.current_waypoint_name not in ("N/A", "Unknown"):
            self.prev_waypoint_name = self.current_waypoint_name

        if self.global_path:
            last = self.global_path[-1]
            self.prev_goal_str = "(%.2f, %.2f)" % (last[0], last[1])

        # Store path from move_base planners for timing and goal name inference
        self.global_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        rospy.loginfo("UI: Received path with %d points" % len(self.global_path))

        if self.global_path:
            goal = self.global_path[-1]
            self.current_goal_str = "(%.2f, %.2f)" % (goal[0], goal[1])
            self.current_waypoint_name = self.find_closest_waypoint_name(goal[0], goal[1])

        if self.latest_map_msg is not None:
            self.path_dirty = True
            self.robot_dirty = True

    def tour_plan_callback(self, msg):
        self.full_plan = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        rospy.loginfo("UI: Received full plan with %d waypoints" % len(self.full_plan))

        self.plan_dirty = True
        self.path_dirty = True
        self.robot_dirty = True

        self.current_seg_index = 0

    def map_callback(self, msg):
        self.latest_map_msg = msg
        self.map_image_dirty = True
        self.path_dirty = True
        self.robot_dirty = True

    def log_callback(self, msg):
        try:
            ts = msg.header.stamp.to_sec()
        except Exception:
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
        path = list(self.global_path)

        if not path or self.robot_x is None:
            return "N/A", "N/A"

        speed = max(self.current_speed_val, 0.05)

        p0x, p0y = path[0]
        curr_dist = math.hypot(p0x - self.robot_x, p0y - self.robot_y)

        total_dist = curr_dist
        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            total_dist += math.hypot(x2 - x1, y2 - y1)

        return "%.1f sec" % (curr_dist / speed), "%.1f sec" % (total_dist / speed)

    # calculates estimated time to a given waypoint index
    def estimate_time_to_waypoint(self, wp_index):
        if self.robot_x is None or self.robot_y is None:
            return None
        if not self.full_plan or wp_index < 0 or wp_index >= len(self.full_plan):
            return None

        # get the speed and apply a safe max to it
        speed = max(self.current_speed_val, 0.05)
        if speed < 0.01: # if we aren't really moving, don't calculate speed
            return None

        if len(self.full_plan) < 2:
            return None

        base_index = min(self.current_seg_index, len(self.full_plan) - 2)

        if wp_index <= base_index:
            return 0.0

        rx, ry = self.robot_x, self.robot_y
        sx, sy = self.full_plan[base_index]

        dist_total = math.hypot(rx - sx, ry - sy)

        for i in range(base_index, wp_index):
            x1, y1 = self.full_plan[i]
            x2, y2 = self.full_plan[i + 1]
            dist_total += math.hypot(x2 - x1, y2 - y1)

        return dist_total / speed

    def format_eta(self, seconds):
        if seconds is None:
            return "N/A"
        if seconds < 0:
            seconds = 0.0
        if seconds < 60.0:
            return "%.1f s" % seconds
        m = int(seconds // 60)
        s = int(seconds % 60)
        return "%d:%02d min" % (m, s)

    def update_map_image(self):
        msg = self.latest_map_msg
        if msg is None:
            return

        w = msg.info.width
        h = msg.info.height
        data = msg.data

        if w == 0 or h == 0 or not data:
            return

        try:
            arr = np.array(data, dtype=np.int8).reshape((h, w))
        except Exception as e:
            rospy.logerr("UI: Failed to reshape map data: %s", str(e))
            return

        qimg = QImage(w, h, QImage.Format_RGB888)
        ptr = qimg.bits()
        ptr.setsize(h * w * 3)
        img_np = np.frombuffer(ptr, dtype=np.uint8).reshape((h, w, 3))

        img_np[arr == 0] = [255, 255, 255]  # free space
        img_np[arr == 100] = [0, 0, 0]      # occupied
        mask_unknown = (arr != 0) & (arr != 100)  # unknown
        img_np[mask_unknown] = [127, 127, 127]    # unknown
        img_np[:] = np.flipud(img_np)             # flip vertically
        pix = QPixmap.fromImage(qimg)

        if self.map_pixmap_item is None:
            self.map_pixmap_item = self.map_scene.addPixmap(pix)
        else:
            self.map_pixmap_item.setPixmap(pix)

        self.map_scene.setSceneRect(0, 0, w, h)

    def update_robot_item(self):
        if self.robot_x is None or self.latest_map_msg is None:
            return

        # Map info
        res = self.latest_map_msg.info.resolution
        ox = self.latest_map_msg.info.origin.position.x
        oy = self.latest_map_msg.info.origin.position.y
        height = self.latest_map_msg.info.height

        # Convert world coords to map pixel coords
        px = (self.robot_x - ox) / res
        py = height - ((self.robot_y - oy) / res)

        # Create robot item if missing
        if self.robot_item is None:
            radius = 6
            color = QColor(160, 32, 240) if not self.robot_frozen else QColor(255, 0, 0)
            self.robot_item = self.map_scene.addEllipse(
                -radius, -radius, radius * 2, radius * 2,
                QPen(Qt.black, 1),
                color
            )
            self.robot_item.setZValue(2)

        # If frozen, do not move robot leave it at last known location and color it red
        if self.robot_frozen:
            self.robot_item.setBrush(QColor(255, 0, 0))  # red
            return

        # Otherwise update robot position normally
        self.robot_item.setPos(px, py)
        self.robot_item.setZValue(2)
        self.robot_item.setBrush(QColor(160, 32, 240))  # active purple

    def update_path_items(self):
        if self.latest_map_msg is None:
            return

        # Map info
        res = self.latest_map_msg.info.resolution
        ox = self.latest_map_msg.info.origin.position.x
        oy = self.latest_map_msg.info.origin.position.y
        height = self.latest_map_msg.info.height

        # Clear old path lines
        for line in self.path_lines:
            self.map_scene.removeItem(line)
        self.path_lines = []

        # Clear old waypoint markers
        for m in self.waypoint_markers:
            self.map_scene.removeItem(m)
        self.waypoint_markers = []

        if len(self.full_plan) < 2:
            # Still update overlays even with no plan
            self.update_time_overlay()
            self.update_freeze_overlay()
            return

        if self.robot_x is not None and self.robot_y is not None:
            if self.current_seg_index >= len(self.full_plan) - 1:
                self.current_seg_index = len(self.full_plan) - 2

            i = self.current_seg_index
            x_curr, y_curr = self.full_plan[i]
            x_next, y_next = self.full_plan[i + 1]
            seg_len = math.hypot(x_next - x_curr, y_next - y_curr)

            if seg_len > 1e-3:
                d_to_next = math.hypot(self.robot_x - x_next, self.robot_y - y_next)
                progress = 1.0 - (d_to_next / seg_len)
                if progress >= 0.85 and self.current_seg_index < len(self.full_plan) - 2:
                    self.current_seg_index += 1

        current_seg = self.current_seg_index

        for i in range(len(self.full_plan) - 1):
            x1, y1 = self.full_plan[i]
            x2, y2 = self.full_plan[i + 1]

            px1 = (x1 - ox) / res
            py1 = height - ((y1 - oy) / res)
            px2 = (x2 - ox) / res
            py2 = height - ((y2 - oy) / res)

            line = QGraphicsLineItem(px1, py1, px2, py2)

            if i < current_seg:
                pen = QPen(QColor(160, 160, 160), 2)
                pen.setStyle(Qt.DotLine)
            elif i == current_seg:
                pen = QPen(QColor(0, 0, 255, 160), 3)
            else:
                pen = QPen(QColor(0, 0, 0), 2)

            line.setPen(pen)
            line.setZValue(1)
            self.map_scene.addItem(line)
            self.path_lines.append(line)

        for idx, (wx, wy) in enumerate(self.full_plan):
            px = (wx - ox) / res
            py = height - ((wy - oy) / res)
            marker = TooltipEllipseItem(px - 3, py - 3, 6, 6)
            marker.setBrush(QColor(0, 255, 0))
            marker.setPen(QPen(Qt.black))
            self.map_scene.addItem(marker)
            marker.setAcceptHoverEvents(True)
            marker.setFlag(QGraphicsItem.ItemIsSelectable, True)
            marker.setZValue(10)

            # Compute waypoint name
            name = self.find_closest_waypoint_name(wx, wy)

            if idx < current_seg:
                status = "Visited waypoint"
                eta_text = ""
            elif idx == current_seg:
                status = "Last passed waypoint"
                eta_text = ""
            elif idx == current_seg + 1:
                status = "Current target waypoint"
                eta_val = self.estimate_time_to_waypoint(idx)
                eta_text = "\nETA: " + self.format_eta(eta_val) if eta_val else "\nETA: N/A"
            else:
                status = "Upcoming waypoint"
                eta_val = self.estimate_time_to_waypoint(idx)
                eta_text = "\nETA: " + self.format_eta(eta_val) if eta_val else "\nETA: N/A"

            tooltip = "%s\n(%.2f, %.2f)\n%s%s" % (
                name, wx, wy, status, eta_text
            )
            marker.setTooltipText(tooltip)

            self.waypoint_markers.append(marker)

        # Map overlays
        self.update_time_overlay()
        self.update_freeze_overlay()

    def update_time_overlay(self):
        if self.tour_duration_sec and self.tour_start_time:
            elapsed = rospy.get_time() - self.tour_start_time
            remaining = max(self.tour_duration_sec - elapsed, 0.0)
            text = "Tour Time Remaining: " + self.format_eta(remaining)
        else:
            text = "Tour Time Remaining: N/A"

        if self.time_text_item is None:
            self.time_text_item = QGraphicsTextItem(text)
            self.time_text_item.setDefaultTextColor(QColor(0, 0, 0))
            self.time_text_item.setZValue(20)
            self.map_scene.addItem(self.time_text_item)
        else:
            self.time_text_item.setPlainText(text)

        rect = self.map_scene.sceneRect()
        self.time_text_item.setPos(rect.left() + 10, rect.top() + 10)

    def update_freeze_overlay(self):
        if not self.robot_frozen:
            if self.freeze_text_item is not None:
                self.map_scene.removeItem(self.freeze_text_item)
                self.freeze_text_item = None
            return

        if self.freeze_text_item is None:
            self.freeze_text_item = self.map_scene.addText("ROBOT FROZEN (Time Expired)")
            self.freeze_text_item.setDefaultTextColor(QColor(255, 0, 0))
            self.freeze_text_item.setZValue(50)

        rect = self.map_scene.sceneRect()
        self.freeze_text_item.setPos(rect.left() + 10, rect.top() + 40)

    def on_timer(self):
        self.current_speed.setText("%.2f m/s" % self.current_speed_val)

        # Always update execution tab text fields
        self.prev_waypoint.setText(self.prev_waypoint_name)
        self.next_waypoint.setText(self.current_waypoint_name)
        t_curr, t_total = self.compute_time_estimates()
        self.time_to_next.setText(t_curr)
        self.time_to_completion.setText(t_total)

        # Tour time remaining (budget)
        remaining = None
        if self.tour_start_time is not None and self.tour_duration_sec is not None:
            elapsed = rospy.get_time() - self.tour_start_time
            remaining = max(self.tour_duration_sec - elapsed, 0.0)
            self.time_remaining.setText(self.format_eta(remaining))
        else:
            self.time_remaining.setText("N/A")

        # If timer expired, stop robot and ask user (once)
        if (
            self.tour_start_time is not None and
            self.tour_duration_sec is not None and
            remaining is not None and
            remaining <= 0.0 and
            not self.timer_alert_shown
        ):
            self.timer_alert_shown = True
            self.robot_frozen = True
            self.update_freeze_overlay()

            # Tell backend to stop robot immediately
            rospy.loginfo("UI: Tour time expired, sending ACTION=STOP_NOW")
            self.pub_tour.publish("ACTION=STOP_NOW")

            # Ask user how to proceed
            choice = QMessageBox.question(
                self,
                "Tour Time Expired",
                "Tour time has elapsed.\n\n"
                "Yes: Continue tour\n"
                "No: Return to starting position",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.Yes
            )
            if choice == QMessageBox.Yes:
                rospy.loginfo("UI: User chose to continue tour.")
                self.pub_tour.publish("ACTION=CONTINUE")
                self.robot_frozen = False
                self.update_freeze_overlay()
            else:
                rospy.loginfo("UI: User chose to return to start.")
                self.pub_tour.publish("ACTION=RETURN")
                # remain frozen visually until new plan starts moving

        # Only update logs if the Logs tab is visible
        if self.logs_dirty and self.tabs.currentIndex() == self.LOG_TAB_INDEX:
            self.update_log_table()
            self.logs_dirty = False

        # Only do heavy map work if simulation tab is visible
        if self.tabs.currentIndex() == self.MAP_TAB_INDEX:
            if self.map_image_dirty:
                self.update_map_image()
                self.map_image_dirty = False
                self.path_dirty = True
                self.robot_dirty = True

            if self.plan_dirty:
                self.path_dirty = True
                self.plan_dirty = False

            if self.path_dirty:
                self.update_path_items()
                self.path_dirty = False

            if self.robot_dirty:
                self.update_robot_item()
                self.robot_dirty = False


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
    timer.start(200)

    app.exec_()

    rospy.signal_shutdown("GUI closed")
