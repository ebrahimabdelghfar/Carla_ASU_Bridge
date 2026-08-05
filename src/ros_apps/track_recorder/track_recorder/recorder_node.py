import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
import math

class TrackRecorderNode(Node):
    def __init__(self):
        super().__init__('track_recorder')
        
        # Declare parameters
        self.declare_parameter('odom_topic', '/sim/odom')
        self.declare_parameter('output_csv', '/tmp/handling_track_recorded.csv')
        self.declare_parameter('w_tr_right_m', 5.0)
        self.declare_parameter('w_tr_left_m', 5.0)
        self.declare_parameter('record_distance_interval', 0.1) # Record every 0.1 meters
        self.declare_parameter('enable_loop_closure', True)
        self.declare_parameter('loop_closure_threshold_m', 3.0) # Close loop when within 3 meters of start
        self.declare_parameter('min_lap_distance_m', 20.0)      # Must travel 20 meters before checking closure
        
        # Get parameters
        odom_topic = self.get_parameter('odom_topic').value
        self.output_csv = self.get_parameter('output_csv').value
        self.w_tr_right_m = self.get_parameter('w_tr_right_m').value
        self.w_tr_left_m = self.get_parameter('w_tr_left_m').value
        self.record_interval = self.get_parameter('record_distance_interval').value
        self.enable_loop_closure = self.get_parameter('enable_loop_closure').value
        self.loop_closure_threshold = self.get_parameter('loop_closure_threshold_m').value
        self.min_lap_distance = self.get_parameter('min_lap_distance_m').value
        
        self.subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            qos_profile_sensor_data
        )
        
        # File setup
        self.file = open(self.output_csv, 'w')
        self.file.write('# x_m,y_m,w_tr_right_m,w_tr_left_m\n')
        
        # Academic Loop Closure State
        self.p0_x = None
        self.p0_y = None
        self.n_x = None
        self.n_y = None
        
        self.last_x = None
        self.last_y = None
        self.prev_proj = 0.0
        self.total_distance = 0.0
        self.is_recording = True
        
        self.get_logger().info(f"Recording from {odom_topic} to {self.output_csv}")

    def write_point(self, x, y):
        self.file.write(f"{x},{y},{self.w_tr_right_m},{self.w_tr_left_m}\n")
        self.file.flush()

    def odom_callback(self, msg):
        if not self.is_recording:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # 1. Record exact Start Point (P0)
        if self.p0_x is None:
            self.p0_x = x
            self.p0_y = y
            self.write_point(x, y)
            self.last_x = x
            self.last_y = y
            return

        dist_from_last = math.sqrt((x - self.last_x)**2 + (y - self.last_y)**2)
        if dist_from_last < self.record_interval:
            return

        # 2. Establish the "Start Line" normal vector from initial movement
        if self.n_x is None:
            dist_from_start = math.sqrt((x - self.p0_x)**2 + (y - self.p0_y)**2)
            self.n_x = (x - self.p0_x) / dist_from_start
            self.n_y = (y - self.p0_y) / dist_from_start
            
            self.total_distance += dist_from_last
            self.write_point(x, y)
            self.last_x = x
            self.last_y = y
            self.prev_proj = dist_from_start
            return

        # 3. Calculate longitudinal projection and lateral drift from start line
        dx = x - self.p0_x
        dy = y - self.p0_y
        proj = dx * self.n_x + dy * self.n_y
        
        if self.enable_loop_closure and self.total_distance > self.min_lap_distance:
            # Check if we crossed the start line (projection goes from negative to positive)
            if self.prev_proj < 0 and proj >= 0:
                # Verify we are laterally close to the start point
                lat_x = dx - proj * self.n_x
                lat_y = dy - proj * self.n_y
                lat_dist = math.sqrt(lat_x**2 + lat_y**2)
                
                if lat_dist < self.loop_closure_threshold:
                    self.get_logger().info(
                        f"🏁 Perfect Loop Closure! Crossed start line after {self.total_distance:.2f}m. "
                        f"Lateral drift: {lat_dist:.2f}m. Appending exact start point and closing."
                    )
                    # Discard current overshoot point, write exact P0 to perfectly close the geometry
                    self.write_point(self.p0_x, self.p0_y)
                    self.is_recording = False
                    self.file.close()
                    self.file = None
                    return
        
        self.total_distance += dist_from_last
        self.write_point(x, y)
        self.last_x = x
        self.last_y = y
        self.prev_proj = proj

    def destroy_node(self):
        if hasattr(self, 'file') and self.file:
            self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TrackRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
