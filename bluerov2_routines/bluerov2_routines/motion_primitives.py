import time
import threading
from mavros_msgs.msg import OverrideRCIn


class MotionPrimitives:
    def __init__(self, ros_node):
        self.node = ros_node
        self.pub = self.node.create_publisher(OverrideRCIn, "/mavros/rc/override", 10)

        self.NEUTRAL = 1500
        self.MAX_DELTA = 400
        self.max_linear = 1.0
        self.max_vertical = 0.7
        self.max_yaw = 1.0

        # Shared state and the lock to protect it
        self.state_lock = threading.Lock()
        self.current_channels = [self.NEUTRAL] * 18

        # Start background loop
        self.stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self.stream_thread.start()

        self.node.get_logger().info("MotionPrimitives ready (Thread-Safe 10Hz Heartbeat)")

    def _pwm(self, value):
        value = max(-1.0, min(1.0, value))
        return int(self.NEUTRAL + value * self.MAX_DELTA)

    def _stream_loop(self):
        """The ONLY loop allowed to publish to MAVROS."""
        while True:
            msg = OverrideRCIn()
            
            # Safely read the shared channel array
            with self.state_lock:
                msg.channels = list(self.current_channels) # Deep copy to ensure thread isolation
                
            self.pub.publish(msg)
            time.sleep(0.1) # Strict 10Hz cadence

    def execute(self, throttle=0.0, yaw=0.0, forward=0.0, lateral=0.0, duration=1.0):
        """Main thread updates the state array, then blocks for the duration."""
        channels = [self.NEUTRAL] * 18
        channels[2] = self._pwm(throttle)   # ch3 vertical
        channels[3] = self._pwm(yaw)        # ch4 yaw
        channels[4] = self._pwm(forward)    # ch5 body X
        channels[5] = self._pwm(lateral)    # ch6 body Y

        # Safely hand the active values to the background thread
        with self.state_lock:
            self.current_channels = channels
        
        # Let the background thread stream these values for the specified duration
        time.sleep(duration)

        # Routine step finished -> Reset back to neutral
        self.stop()

    def stop(self):
        """Resets the shared state array to neutral values."""
        with self.state_lock:
            self.current_channels = [self.NEUTRAL] * 18

    # ==========================================================
    # PRIMITIVES (Identical signatures for full compatibility)
    # ==========================================================
    def forward(self, duration, speed):
        self.node.get_logger().info("FORWARD")
        self.execute(forward=speed * self.max_linear, duration=duration)

    def backward(self, duration, speed):
        self.node.get_logger().info("BACKWARD")
        self.execute(forward=-speed * self.max_linear, duration=duration)

    def left(self, duration, speed):
        self.node.get_logger().info("LEFT")
        self.execute(lateral=speed * self.max_linear, duration=duration)

    def right(self, duration, speed):
        self.node.get_logger().info("RIGHT")
        self.execute(lateral=-speed * self.max_linear, duration=duration)

    def ascend(self, duration, speed):
        self.node.get_logger().info("ASCEND")
        self.execute(throttle=speed * self.max_vertical, duration=duration)

    def descend(self, duration, speed):
        self.node.get_logger().info("DESCEND")
        self.execute(throttle=-speed * self.max_vertical, duration=duration)

    def rotate_left(self, duration, speed):
        self.node.get_logger().info("ROTATE LEFT")
        self.execute(yaw=speed * self.max_yaw, duration=duration)

    def rotate_right(self, duration, speed):
        self.node.get_logger().info("ROTATE RIGHT")
        self.execute(yaw=-speed * self.max_yaw, duration=duration)