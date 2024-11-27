import sys
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QPushButton, QLabel, QWidget
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import String


class UserInterface(Node):
    def __init__(self):
        super().__init__('user_interface')

        # ROS 2 Publishers and Subscribers
        self.command_pub = self.create_publisher(String, '/user_command', 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/bot5/map', self.map_callback, 10)

        # TF Listener for robot position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ROS 2 Timer for GUI Updates
        self.create_timer(0.1, self.update_gui)

        # GUI Application
        self.app = QApplication(sys.argv)
        self.window = QWidget()
        self.window.setWindowTitle("SLAM User Interface")
        self.layout = QVBoxLayout()

        # Status Label
        self.status_label = QLabel("SLAM 상태: 탐색 중...")
        self.layout.addWidget(self.status_label)

        # Buttons
        self.stop_and_return_button = QPushButton("탐색 중지 후 Base로 복귀")
        self.resume_exploration_button = QPushButton("탐색 재개")
        self.activate_teleop_button = QPushButton("Teleop 활성화")
        self.random_exploration_button = QPushButton("랜덤 탐험 실행")
        self.end_exploration_button = QPushButton("탐험 종료")

        # Button Events
        self.stop_and_return_button.clicked.connect(self.stop_and_return)
        self.resume_exploration_button.clicked.connect(self.resume_exploration)
        self.activate_teleop_button.clicked.connect(self.activate_teleop)
        self.random_exploration_button.clicked.connect(self.random_exploration)
        self.end_exploration_button.clicked.connect(self.end_exploration)

        # Add Buttons to Layout
        self.layout.addWidget(self.stop_and_return_button)
        self.layout.addWidget(self.resume_exploration_button)
        self.layout.addWidget(self.activate_teleop_button)
        self.layout.addWidget(self.random_exploration_button)
        self.layout.addWidget(self.end_exploration_button)

        # Map Visualization
        self.figure, self.ax = plt.subplots()
        self.canvas = FigureCanvas(self.figure)
        self.layout.addWidget(self.canvas)

        # Initialize map and robot position
        self.map_array = None
        self.robot_position = None

        # Set Layout
        self.window.setLayout(self.layout)
        self.window.show()

    def stop_and_return(self):
        """Handle Stop and Return Command."""
        self.command_pub.publish(String(data="stop_and_return"))
        self.status_label.setText("SLAM 상태: Base로 복귀 중...")
        self.get_logger().info("Command sent: Stop and return to base.")

    def resume_exploration(self):
        """Handle Resume Exploration Command."""
        self.command_pub.publish(String(data="resume_exploration"))
        self.status_label.setText("SLAM 상태: 탐색 재개...")
        self.get_logger().info("Command sent: Resume exploration.")

    def activate_teleop(self):
        """Handle Activate Teleop Command."""
        self.command_pub.publish(String(data="activate_teleop"))
        self.status_label.setText("SLAM 상태: Teleop 모드 활성화...")
        self.get_logger().info("Command sent: Activate teleop mode.")

    def random_exploration(self):
        """Handle Random Exploration Command."""
        self.command_pub.publish(String(data="random_exploration"))
        self.status_label.setText("SLAM 상태: 랜덤 탐험 실행 중...")
        self.get_logger().info("Command sent: Random exploration.")

    def end_exploration(self):
        """Handle End Exploration Command."""
        self.command_pub.publish(String(data="end_exploration"))
        self.status_label.setText("SLAM 상태: 탐험 종료.")
        self.get_logger().info("Command sent: End exploration.")

    def map_callback(self, msg):
        """Callback to process incoming map data and update visualization."""
        self.get_logger().info("Map data received.")
        # Convert OccupancyGrid to NumPy array
        self.map_array = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)

    def get_robot_position(self):
        """Get the robot's position in the 'map' frame using TF."""
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
            return (transform.transform.translation.x, transform.transform.translation.y)
        except Exception as e:
            self.get_logger().error(f"Could not get robot position: {e}")
            return None

    def update_gui(self):
        """Update GUI and refresh the map visualization."""
        if self.map_array is not None:
            self.robot_position = self.get_robot_position()
            self.ax.clear()

            # Display the map
            self.ax.imshow(self.map_array, cmap='gray', origin='lower')

            # Display the robot position
            if self.robot_position:
                x, y = self.robot_position
                # Convert robot position to map indices
                x_idx = int((x - self.map_origin[0]) / self.map_resolution)
                y_idx = int((y - self.map_origin[1]) / self.map_resolution)
                self.ax.plot(y_idx, x_idx, 'ro', label="Robot Position")  # Plot as a red dot

            self.ax.legend()
            self.canvas.draw()

    def run(self):
        """Run the GUI application."""
        sys.exit(self.app.exec_())


def main(args=None):
    rclpy.init(args=args)
    node = UserInterface()
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
