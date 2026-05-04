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
    
    from moveit_msgs.srv import GetPositionIK, GetPositionFK
    from moveit_msgs.msg import PositionIKRequest

    
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


class RosWorker(QThread):
#===============================================================
 #this added
    tcp_pose_received = pyqtSignal(float, float, float, float, float, float)
    # --- YENİ SİNYALLER (GHOST SYNC) ---
    ghost_joints_received = pyqtSignal(list)
    ghost_tcp_received = pyqtSignal(float, float, float, float, float, float)
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

        # --- YENİ EKLENEN: Dinamik Limit Değişkenleri ---
        self.limit_joint_speed = 1.0   # 1.0 = %100 hız
        self.limit_tcp_speed = 2500.0  # mm/s     # 1.0 = %100 tork
        self.limit_collision = 0.5     # 0.5 = %50 tolerans
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
            self._pub_g = self._node.create_publisher(
                JointTrajectory, "/gripper_controller/joint_trajectory", 10)
            self._pub_t = self._node.create_publisher(Twist,  "/cmd_vel", 10)
            self._pub_c = self._node.create_publisher(String, "/robot_command", 10)
            
            self._pub_ghost = self._node.create_publisher(
                DisplayRobotState, '/display_robot_state', 10) # MoveIt için hayalet robot yayıncısı
            
            self.move_action_client = ActionClient(self._node, MoveGroup, 'move_action')
            self._ik_client = self._node.create_client(GetPositionIK, '/compute_ik')
            self._fk_client = self._node.create_client(GetPositionFK, '/compute_fk') 
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
    def set_limits(self, joint_speed_pct: float, tcp_speed_mm: float , collision_pct: float):
        """Arayüzden gelen limit değerlerini günceller."""
        with self._lock:
            # ROS 2 hata vermesin diye kesinlikle float tipine çeviriyoruz (0.0 ile 1.0 arası)
            self.limit_joint_speed = float(joint_speed_pct) / 100.0
            self.limit_tcp_speed = float(tcp_speed_mm)
            self.limit_collision = float(collision_pct) / 100.0
            
        self.log_message.emit(f"Hız limitleri güncellendi (Hız: %{joint_speed_pct})", "warn")
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
        # --- YENİ KOD: Dinamik Hız Ölçeklendirme ---
        with self._lock:
            current_speed = self.limit_joint_speed
            
        goal_msg.request.max_velocity_scaling_factor = current_speed      
        goal_msg.request.max_acceleration_scaling_factor = current_speed 
        # -------------------------------------------
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
        # --- YENİ KOD: Dinamik Hız Ölçeklendirme ---
        with self._lock:
            current_speed = self.limit_joint_speed
            
        goal_msg.request.max_velocity_scaling_factor = current_speed      
        goal_msg.request.max_acceleration_scaling_factor = current_speed 
        # -------------------------------------------
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

    def publish_cmd(self, cmd: str): # 
        """String komutunu /robot_command'a gönder."""
        if not self._pub_c:
            return
        msg = String()
        msg.data = cmd
        with self._lock:
            self._pub_c.publish(msg)
            
    def publish_ghost_robot(self, angles_rad: list):
        """Arayüzdeki anlık slider konumlarını RViz2'deki hayalet robota gönderir ve FK hesaplar."""
        if not hasattr(self, '_pub_ghost') or not self._pub_ghost:
            return
        
        msg = DisplayRobotState()
        js = JointState()
        js.name = self.JOINT_NAMES
        js.position = angles_rad
        
        msg.state.joint_state = js
        
        with self._lock:
            self._pub_ghost.publish(msg)

        # FK (İleri Kinematik) İsteyerek TCP sliderlarını güncelle
        if not hasattr(self, '_fk_client') or not self._fk_client.wait_for_service(timeout_sec=0.05):
            return
            
        req = GetPositionFK.Request()
        req.header.frame_id = self._base_frame
        req.fk_link_names = [self._tool_frame]
        req.robot_state.joint_state = js
        
        future = self._fk_client.call_async(req)
        future.add_done_callback(self._fk_callback)

    def _fk_callback(self, future):
        """FK çözümü geldiğinde TCP slider'larını güncellemek için sinyal gönderir."""
        try:
            response = future.result()
            if response.error_code.val == 1 and len(response.pose_stamped) > 0:
                pose = response.pose_stamped[0].pose
                q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
                roll, pitch, yaw = euler_from_quaternion(q)
                
                # UI'a Metre -> Milimetre, Radyan -> Derece olarak gönder
                self.ghost_tcp_received.emit(
                    pose.position.x * 1000.0,
                    pose.position.y * 1000.0,
                    pose.position.z * 1000.0,
                    math.degrees(roll),
                    math.degrees(pitch),
                    math.degrees(yaw)
                )
        except Exception:
            pass
    def publish_cartesian_ghost(self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float):
        """Kartezyen hedef için IK hesaplar ve sonucu Hayalet Robot olarak RViz'e gönderir."""
        if not hasattr(self, '_ik_client') or not self._ik_client.wait_for_service(timeout_sec=0.05):
            return
        
        req = GetPositionIK.Request()
        ik_req = PositionIKRequest()
        ik_req.group_name = "tm5_arm"
        ik_req.pose_stamped.header.frame_id = self._base_frame
        
        # Pozisyon (Metre)
        ik_req.pose_stamped.pose.position.x = x
        ik_req.pose_stamped.pose.position.y = y
        ik_req.pose_stamped.pose.position.z = z
        
        # Yönelim (Quaternion)
        q = quaternion_from_euler(roll, pitch, yaw)
        ik_req.pose_stamped.pose.orientation.x = q[0]
        ik_req.pose_stamped.pose.orientation.y = q[1]
        ik_req.pose_stamped.pose.orientation.z = q[2]
        ik_req.pose_stamped.pose.orientation.w = q[3]
        
        req.ik_request = ik_req
        
        # UI donmasın diye arka planda (asenkron) çözümü iste
        future = self._ik_client.call_async(req)
        future.add_done_callback(self._ik_callback)
        
    def _ik_callback(self, future):
        """IK çözümü geldiğinde çalışır, Hayalet Robotu çizer ve Joint slider'larını günceller."""
        try:
            response = future.result()
            # 1 = SUCCESS (MoveIt başarı kodu)
            if response.error_code.val == 1:
                msg = DisplayRobotState()
                msg.state = response.solution
                with self._lock:
                    if self._pub_ghost:
                        self._pub_ghost.publish(msg)
                
                # IK çözümündeki joint açılarını arayüze gönder
                joint_positions = list(response.solution.joint_state.position)
                self.ghost_joints_received.emit(joint_positions)
        except Exception:
            pass

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

    def set_gripper(self, action: str):
        """Gripper'ı fiziksel olarak açıp kapatmak için doğrudan motorlara komut gönderir."""
        if not hasattr(self, '_pub_g') or not self._pub_g:
            self.log_message.emit("Gripper publisher bulunamadı!", "err")
            return
            
        msg = JointTrajectory()
        
        # Her iki parmağı da motor olarak kontrol ediyoruz
        msg.joint_names = ["lefthand_joint", "righthand_joint"]
        
        pt = JointTrajectoryPoint()
        
        # Her iki parmağa aynı anda aynı mesafeyi gönderiyoruz
        if action == "OPEN":
            pt.positions = [-0.015, -0.015]   
        elif action == "CLOSE":
            pt.positions = [0.025, 0.025] 
            
        pt.time_from_start = Duration(sec=1, nanosec=0)
        msg.points = [pt]
        
        with self._lock:
            self._pub_g.publish(msg)