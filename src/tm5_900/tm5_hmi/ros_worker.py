# ─────────────────────────────────────────────────────────────────────────────
#  ros_worker.py  —  ROS 2 iletişim iş parçacığı
#  Pub/Sub: /joint_states  |  /tm_arm_controller/joint_trajectory
#           /cmd_vel       |  /robot_command
# ─────────────────────────────────────────────────────────────────────────────
#===============================================================
 #this added
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion
#===============================================================
import math
import threading

from PyQt5.QtCore import QThread, pyqtSignal

from config import TM5_JOINTS

# ── ROS 2 opsiyonel import (kurulu değilse demo mod) ─────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


class RosWorker(QThread):
#===============================================================
 #this added
    tcp_pose_received = pyqtSignal(float, float, float, float, float, float)
 #===============================================================
    
    joint_state_received = pyqtSignal(list, list, list)   # pos, vel, eff
    connection_changed   = pyqtSignal(bool)
    log_message          = pyqtSignal(str, str)            # message, kind

    JOINT_NAMES = [j["ros"] for j in TM5_JOINTS]

    def __init__(self):
        super().__init__()
        self._node   = None
        self._active = True
        self._pub_j  = None   # JointTrajectory publisher
        self._pub_t  = None   # Twist publisher
        self._pub_c  = None   # String command publisher
        self._lock   = threading.Lock()
 #===============================================================
 #this added
        self._tf_buffer = None
        self._tf_listener = None
        self._base_frame = "base_link"
        self._tool_frame = "link6"
 #===============================================================
    # ── Ana döngü ─────────────────────────────────────────────────────────────
    def run(self):
        if not ROS_AVAILABLE:
            self.connection_changed.emit(False)
            self.log_message.emit("rclpy bulunamadı — demo modu", "warn")
            return
        try:
            rclpy.init()
            self._node  = Node("tm5_hmi")
 #===============================================================
#this added
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self._node)
 #===============================================================
  
            self._pub_j = self._node.create_publisher(
                JointTrajectory, "/arm_controller/joint_trajectory", 10)
            self._pub_t = self._node.create_publisher(Twist,  "/cmd_vel", 10)
            self._pub_c = self._node.create_publisher(String, "/robot_command", 10)
            self._node.create_subscription(
                JointState, "/joint_states", self._js_cb, 10)

            self.connection_changed.emit(True)
            self.log_message.emit("ROS 2 bağlandı — tm5_hmi node aktif", "ok")

            while self._active and rclpy.ok():
                rclpy.spin_once(self._node, timeout_sec=0.05)
 #===============================================================
  #this added
                try:
                    tf = self._tf_buffer.lookup_transform(
                    self._base_frame,
                    self._tool_frame,
                    rclpy.time.Time()
      )

                    t = tf.transform.translation
                    q = tf.transform.rotation

                    roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

                    x = t.x * 1000.0
                    y = t.y * 1000.0
                    z = t.z * 1000.0
                    rx = math.degrees(roll)
                    ry = math.degrees(pitch)
                    rz = math.degrees(yaw)

                    self.tcp_pose_received.emit(x, y, z, rx, ry, rz)

                except Exception:
                    pass
 #===============================================================
        except Exception as exc:
            self.connection_changed.emit(False)
            self.log_message.emit(f"ROS 2 hatası: {exc}", "err")

    # ── /joint_states callback ────────────────────────────────────────────────
    def _js_cb(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        pos, vel, eff = [], [], []
        for jn in self.JOINT_NAMES:
            i = idx.get(jn)
            pos.append(msg.position[i] if i is not None and msg.position else 0.0)
            vel.append(msg.velocity[i] if i is not None and msg.velocity else 0.0)
            eff.append(msg.effort[i]   if i is not None and msg.effort   else 0.0)
        self.joint_state_received.emit(pos, vel, eff)

    # ── Publish yardımcıları ──────────────────────────────────────────────────
    def publish_joints(self, angles_deg: list):
        """Derece cinsinden 6 eklem açısını JointTrajectory olarak yayınla."""
        if not self._pub_j:
            return
        n  = len(TM5_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions       = [math.radians(a) for a in angles_deg[:n]]
        pt.velocities      = [0.0] * n
        pt.accelerations   = [0.0] * n
        pt.time_from_start = Duration(sec=1, nanosec=0)
        msg = JointTrajectory()
        msg.joint_names = self.JOINT_NAMES
        msg.points      = [pt]
        with self._lock:
            self._pub_j.publish(msg)

    def publish_twist(self, lx: float, ly: float, lz: float, az: float):
        """Kartezyen hız komutunu /cmd_vel'e gönder."""
        if not self._pub_t:
            return
        msg = Twist()
        msg.linear.x  = lx
        msg.linear.y  = ly
        msg.linear.z  = lz
        msg.angular.z = az
        with self._lock:
            self._pub_t.publish(msg)

    def publish_cmd(self, cmd: str):
        """String komutunu /robot_command'a gönder."""
        if not self._pub_c:
            return
        msg = String()
        msg.data = cmd
        with self._lock:
            self._pub_c.publish(msg)

    # ── Temiz kapatma ─────────────────────────────────────────────────────────
    def stop(self):
        self._active = False
        try:
            if self._node:
                self._node.destroy_node()
        except Exception:
            pass
        try:
            if ROS_AVAILABLE and rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
