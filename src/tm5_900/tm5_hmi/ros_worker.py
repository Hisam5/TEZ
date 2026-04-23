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
    
    from rclpy.action import ActionClient
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import Constraints, JointConstraint
    
    from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
    from shape_msgs.msg import SolidPrimitive
    from geometry_msgs.msg import Pose
    from tf_transformations import quaternion_from_euler
    
    from moveit_msgs.msg import DisplayRobotState, RobotState
    from sensor_msgs.msg import JointState
    
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
        
        self.move_action_client = None #MoveIt Client referansı
        
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
            
            self._pub_ghost = self._node.create_publisher(
                DisplayRobotState, '/display_robot_state', 10) # MoveIt için hayalet robot yayıncısı
            
            self.move_action_client = ActionClient(self._node, MoveGroup, 'move_action')
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
    def send_moveit_goal(self, joint_angles_rad: list):
        """Arayüzden gelen radyan cinsi açıları MoveIt'e güvenli planlama için gönderir."""
        if not self.move_action_client:
            self.log_message.emit("MoveIt Action Client tanımlı değil!", "err")
            return

        # MoveIt sunucusunun açık olup olmadığını kontrol et
        if not self.move_action_client.wait_for_server(timeout_sec=1.0):
            self.log_message.emit("MoveIt 'move_action' sunucusu bulunamadı!", "err")
            return

        # --- EKLENEN KISIM BAŞLANGICI: MoveIt Yörünge Hedefi Gönderme Metodu ---
        #Jointlarin hiz limitleri scale yapilmis olarak gonderilir, bu sayede arayuzden gelen komutlar guvenli hale gelir
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "tm5_arm" # SRDF dosyasındaki grup adı
        goal_msg.request.num_planning_attempts = 3
        goal_msg.request.allowed_planning_time = 3.0
        goal_msg.request.max_velocity_scaling_factor = 0.5      # %50 Hız (Güvenlik için)
        goal_msg.request.max_acceleration_scaling_factor = 0.5  # %50 İvme

        constraints = Constraints() 
        for i, angle in enumerate(joint_angles_rad): # Gelen radyan cinsinden açıları MoveIt'in JointConstraint formatına dönüştür
            jc = JointConstraint()
            jc.joint_name = self.JOINT_NAMES[i]
            jc.position = angle
            jc.tolerance_above = 0.005 # Hassasiyet toleransları
            jc.tolerance_below = 0.005
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal_msg.request.goal_constraints.append(constraints)

        self.log_message.emit(f"MoveIt hedefe gidiyor...", "ok")
        # Planı asenkron (arayüzü dondurmadan) gönder
        self.move_action_client.send_goal_async(goal_msg)
        # --- EKLENEN KISIM SONU ---
        
    def send_moveit_cartesian_goal(self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float):
        """Arayüzden gelen Kartezyen (XYZ + RPY) değerleri MoveIt Pose hedefine dönüştürüp gönderir."""
        if not self.move_action_client:
            self.log_message.emit("MoveIt Action Client tanımlı değil!", "err")
            return
        if not self.move_action_client.wait_for_server(timeout_sec=1.0):
            self.log_message.emit("MoveIt 'move_action' sunucusu bulunamadı!", "err")
            return
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "tm5_arm" 
        goal_msg.request.num_planning_attempts = 3
        goal_msg.request.allowed_planning_time = 3.0
        goal_msg.request.max_velocity_scaling_factor = 0.5     
        goal_msg.request.max_acceleration_scaling_factor = 0.5 
        constraints = Constraints()
        # 1. Position Constraint (XYZ)
        pc = PositionConstraint()
        pc.header.frame_id = self._base_frame
        pc.link_name = self._tool_frame
        
        # Hedef pozisyon için tolerans alanı (küre)
        bv = BoundingVolume()
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.SPHERE
        prim.dimensions = [0.0005]  # .5mm pozisyon toleransı
        bv.primitives.append(prim)
        
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0 # Geçici yönelim (sadece pozisyon bölgesini tanımlamak için)
        bv.primitive_poses.append(pose)
        
        pc.constraint_region = bv
        pc.weight = 1.0
        constraints.position_constraints.append(pc)
        # 2. Orientation Constraint (RPY -> Quaternion)
        oc = OrientationConstraint()
        oc.header.frame_id = self._base_frame
        oc.link_name = self._tool_frame
        
        # Euler (Roll, Pitch, Yaw) açılarını Quaternion'a (x, y, z, w) çevir
        q = quaternion_from_euler(roll, pitch, yaw)
        oc.orientation.x = q[0]
        oc.orientation.y = q[1]
        oc.orientation.z = q[2]
        oc.orientation.w = q[3]
        
        # Açısal toleranslar (radyan cinsinden, yakl. 2.8 derece)
        oc.absolute_x_axis_tolerance = 0.005# DEĞİŞTİRİLDİ
        oc.absolute_y_axis_tolerance = 0.005 # DEĞİŞTİRİLDİ
        oc.absolute_z_axis_tolerance = 0.005 # DEĞİŞTİRİLDİ
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)
        goal_msg.request.goal_constraints.append(constraints)
        self.log_message.emit(f"MoveIt Kartezyen hedefe (IK) gidiyor...", "ok")
        self.move_action_client.send_goal_async(goal_msg)

    
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
            
    def publish_ghost_robot(self, angles_rad: list): # MoveIt için hayalet robot yayınlama metodu
        """Arayüzdeki anlık slider konumlarını RViz2'deki hayalet robota gönderir."""
        if not hasattr(self, '_pub_ghost') or not self._pub_ghost:
            return
        
        msg = DisplayRobotState()
        js = JointState()
        js.name = self.JOINT_NAMES
        js.position = angles_rad
        
        msg.state.joint_state = js
        
        with self._lock:
            self._pub_ghost.publish(msg)

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
