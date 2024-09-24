import sys
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from visualization_msgs.msg import MarkerArray
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton
from PyQt5.QtGui import QPainter, QColor
from PyQt5.QtCore import QRectF, Qt

class MarkerControlNode(Node):
    def __init__(self):
        super().__init__('marker_control_node')
        qos_profile = rclpy.qos.QoSProfile(depth=10)
        self.cell_state_publisher = self.create_publisher(Int32, 'cell_state_change', qos_profile)
        self.marker_subscriber = self.create_subscription(MarkerArray, 'visualization_marker_array', self.marker_callback, qos_profile)
        self.cell_states = {}
        self.latest_marker_data = None  # To store the latest MarkerArray data

    def update_cell_state(self, cell_id):
        msg = Int32()
        msg.data = cell_id
        self.cell_state_publisher.publish(msg)
        self.get_logger().info(f'Published cell {cell_id} state change')

    def marker_callback(self, msg: MarkerArray):
        self.latest_marker_data = msg  # Store the latest MarkerArray data
        self.cell_states.clear()
        for marker in msg.markers:
            if marker.ns == "midpoints":
                cell_id = marker.id
                # Update cell state based on color (green for active, red for inactive)
                self.cell_states[cell_id] = marker.color.r == 0.0
        self.get_logger().info("Received updated marker data from RViz")

class MarkerControlGUI(QWidget):
    def __init__(self, cells, midpoints, resolution, origin, ros_node):
        super().__init__()
        self.cells = cells
        self.midpoints = midpoints
        self.resolution = resolution
        self.origin = origin
        self.ros_node = ros_node
        self.cell_states = {i: True for i in range(len(cells))}
        self.midpoint_states = {i: True for i in range(len(midpoints))}
        self.is_dragging = False
        self.last_clicked_cell = None
        self.initUI()
        self.load_initial_states()

    def initUI(self):
        self.setGeometry(100, 100, 800, 600)
        self.setWindowTitle('Cell Control')
        self.setMouseTracking(True)

        layout = QVBoxLayout()
        
        self.refresh_button = QPushButton('Refresh', self)
        self.refresh_button.clicked.connect(self.refresh_states)
        layout.addWidget(self.refresh_button)
        
        self.setLayout(layout)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Draw cells
        for i, cell in enumerate(self.cells):
            x = (cell['x'] - self.origin[0]) / self.resolution
            y = (cell['y'] - self.origin[1]) / self.resolution
            width = cell['width'] / self.resolution
            height = cell['height'] / self.resolution
            
            if self.cell_states.get(i, True):
                painter.setBrush(QColor(0, 255, 0, 127))  # Green with transparency
            else:
                painter.setBrush(QColor(255, 0, 0, 127))  # Red with transparency
            painter.drawRect(int(x), int(y), int(width), int(height))

        # Draw midpoints
        for i, midpoint in enumerate(self.midpoints):
            x = (midpoint['x'] - self.origin[0]) / self.resolution
            y = (midpoint['y'] - self.origin[1]) / self.resolution
            radius = min(self.get_cell_width(midpoint), self.get_cell_height(midpoint)) / 4
            
            if self.midpoint_states.get(i, True):
                painter.setBrush(QColor(0, 255, 0, 127))  # Green with transparency
            else:
                painter.setBrush(QColor(255, 0, 0, 127))  # Red with transparency
            painter.drawEllipse(int(x - radius), int(y - radius), int(radius * 2), int(radius * 2))

    def get_cell_width(self, midpoint):
        cell_idx = self.find_cell_for_midpoint(midpoint)
        if cell_idx is not None:
            cell = self.cells[cell_idx]
            return cell['width']
        return 0

    def get_cell_height(self, midpoint):
        cell_idx = self.find_cell_for_midpoint(midpoint)
        if cell_idx is not None:
            cell = self.cells[cell_idx]
            return cell['height']
        return 0

    def find_cell_for_midpoint(self, midpoint):
        for i, cell in enumerate(self.cells):
            if (cell['x'] <= midpoint['x'] < cell['x'] + cell['width'] and
                cell['y'] <= midpoint['y'] < cell['y'] + cell['height']):
                return i
        return None

    def mousePressEvent(self, event):
        self.is_dragging = True
        pos = event.pos()
        self.check_click(pos)

    def mouseMoveEvent(self, event):
        if self.is_dragging:
            pos = event.pos()
            self.check_click(pos)

    def mouseReleaseEvent(self, event):
        self.is_dragging = False
        self.last_clicked_cell = None  # Reset the last clicked cell

    def check_click(self, pos):
        x = pos.x()
        y = pos.y()
        for i, cell in enumerate(self.cells):
            cell_rect = QRectF(
                (cell['x'] - self.origin[0]) / self.resolution,
                (cell['y'] - self.origin[1]) / self.resolution,
                cell['width'] / self.resolution,
                cell['height'] / self.resolution
            )
            if cell_rect.contains(x, y):
                if self.is_dragging:
                    if self.last_clicked_cell != i:  # Change state only if cell is different from last clicked
                        if i in self.cell_states:
                            self.cell_states[i] = not self.cell_states[i]
                        else:
                            self.cell_states[i] = True
                        self.midpoint_states[i] = self.cell_states[i]
                        self.ros_node.update_cell_state(i)
                        self.last_clicked_cell = i  # Update the last clicked cell
                        self.update()
                break

    def load_initial_states(self):
        # Load initial states from RViz
        self.ros_node.get_logger().info("Loading initial states from RViz...")
        # Check if marker data is available and update states
        if self.ros_node.latest_marker_data:
            for marker in self.ros_node.latest_marker_data.markers:
                if marker.ns == "midpoints":
                    cell_id = marker.id
                    self.cell_states[cell_id] = marker.color.r == 0.0
                    self.midpoint_states[cell_id] = marker.color.r == 0.0
            self.update()

    def refresh_states(self):
        # Refresh states by requesting updates from RViz
        self.ros_node.get_logger().info("Refreshing states from RViz...")
        if self.ros_node.latest_marker_data:
            for marker in self.ros_node.latest_marker_data.markers:
                if marker.ns == "midpoints":
                    cell_id = marker.id
                    self.cell_states[cell_id] = marker.color.r == 0.0
                    self.midpoint_states[cell_id] = marker.color.r == 0.0
            self.update()  # Force an update to reflect the new states
        else:
            self.ros_node.get_logger().warning("No marker data available for refresh")

def main():
    rclpy.init()
    ros_node = MarkerControlNode()
    app = QApplication(sys.argv)
    with open('cell_midpoint.json', 'r') as f:
        data = json.load(f)
    cells = data['cells']
    midpoints = data['midpoints']
    resolution = 0.05
    origin = [-5.75, -5.06]
    gui = MarkerControlGUI(cells, midpoints, resolution, origin, ros_node)
    gui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
