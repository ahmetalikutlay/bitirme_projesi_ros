#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
GRADUATION PROJECT - GROUP 1
Istanbul Aydin University - Industrial Engineering
Development of an Autonomous ROS-Based Robotic Arm for Recycling

Team:
  - Ahmet Ali Kutlay    (B2105.030074) - IE: KPI, Experimental Design, Report
  - Ertuğrul Başören   (B2105.010129) - Vision System, Object Detection
  - Çağatay Özcandan   (B2105.010111) - Motion Planning, MoveIt Integration
  - Levent Arda Padar  (B2180.060064) - Simulation, ROS Infrastructure, Dashboard
"""

import rospy
import threading
import csv
import time
import tf
import cv2
import numpy as np
import sys

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_msgs.msg import String

# MoveIt
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface

# ── KPI LOGGING ──────────────────────────────────────────────────────────────
KPI_FILE = '/tmp/kpi_log.csv'

def init_kpi_log():
    with open(KPI_FILE, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['trial_no', 'atik_tipi', 'tespit_dogru', 'cycle_time_s', 'grasp_basarili'])
    print(f">>> KPI log dosyasi olusturuldu: {KPI_FILE}")

def log_kpi(trial_no, atik_tipi, tespit_dogru, cycle_time_s, grasp_basarili):
    with open(KPI_FILE, 'a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([trial_no, atik_tipi, tespit_dogru, round(cycle_time_s, 3), grasp_basarili])

# ─────────────────────────────────────────────────────────────────────────────

class SortingRobot:
    def __init__(self):
        # MoveIt commander başlat
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('bitirme_projesi_sorting')

        # MoveIt
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.arm = MoveGroupCommander("manipulator")

        # MoveIt ayarları
        self.arm.set_max_velocity_scaling_factor(0.5)
        self.arm.set_max_acceleration_scaling_factor(0.3)
        self.arm.set_planning_time(15.0)
        self.arm.set_num_planning_attempts(10)
        self.arm.allow_replanning(True)

        # Status publisher (dashboard için)
        self.status_pub = rospy.Publisher('/robot_status', String, queue_size=10)
        self.detection_pub = rospy.Publisher('/detected_color', String, queue_size=10)

        # KPI
        init_kpi_log()
        self.trial_no = 0
        self.cycle_start_time = None

        # Vision
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.camera_callback)
        self.detected_type = None
        self.is_busy = False

        # Gazebo model service
        rospy.wait_for_service('/gazebo/set_model_state')
        self.box_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.magnet_active = False
        self.target_box_name = ''
        self.tf_listener = tf.TransformListener()

        # Magnet thread
        self.worker = threading.Thread(target=self.magnet_thread)
        self.worker.daemon = True
        self.worker.start()

        print(">>> ATIK AYRISTIRMA SISTEMI BASLATILIYOR <<<")
        print(">>> MoveIt hazirlanıyor...")
        rospy.sleep(2.0)

        self.go_home_pose()
        self.publish_status("HAZIR")

        print("\n" + "="*55)
        print(" BEKLENIYOR: Kirmizi / Mavi / Yesil / Gri Kutu")
        print(" Plastik  -> Kirmizi  -> Sol")
        print(" Cam      -> Mavi     -> Sag")
        print(" Kagit    -> Yesil    -> Ileri Sol")
        print(" Metal    -> Gri      -> Ileri Sag")
        print("="*55 + "\n")

        rospy.spin()

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def camera_callback(self, data):
        if self.is_busy:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # ── 1. KIRMIZI (Plastik) ──
            mask_red = (
                cv2.inRange(hsv, np.array([0, 70, 50]),   np.array([10, 255, 255])) +
                cv2.inRange(hsv, np.array([170, 70, 50]), np.array([180, 255, 255]))
            )

            # ── 2. MAVİ (Cam) ──
            mask_blue = cv2.inRange(hsv, np.array([100, 150, 0]), np.array([140, 255, 255]))

            # ── 3. YEŞİL (Kağıt) ──
            mask_green = cv2.inRange(hsv, np.array([40, 80, 50]), np.array([80, 255, 255]))

            # ── 4. GRİ (Metal) ──
            mask_gray = cv2.inRange(hsv, np.array([0, 0, 60]), np.array([180, 50, 200]))

            # Gurultu filtrele
            kernel = np.ones((5, 5), np.uint8)
            mask_red   = cv2.morphologyEx(mask_red,   cv2.MORPH_OPEN, kernel)
            mask_blue  = cv2.morphologyEx(mask_blue,  cv2.MORPH_OPEN, kernel)
            mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
            mask_gray  = cv2.morphologyEx(mask_gray,  cv2.MORPH_OPEN, kernel)

            # Gorsellestir
            self.draw_box(frame, mask_red,   (0, 0, 255),     "PLASTIK (Kirmizi)")
            self.draw_box(frame, mask_blue,  (255, 0, 0),     "CAM (Mavi)")
            self.draw_box(frame, mask_green, (0, 255, 0),     "KAGIT (Yesil)")
            self.draw_box(frame, mask_gray,  (128, 128, 128), "METAL (Gri)")

            cv2.imshow("AYRISTIRMA KAMERASI", frame)
            cv2.waitKey(1)

            THRESHOLD = 3000
            if cv2.countNonZero(mask_red) > THRESHOLD:
                print(">>> TESPIT: KIRMIZI (Plastik) -> SOL")
                self.target_box_name = 'kutu_kirmizi'
                self.detected_type = 'RED'
                self.detection_pub.publish('RED')
                self.start_sorting_thread()
            elif cv2.countNonZero(mask_blue) > THRESHOLD:
                print(">>> TESPIT: MAVI (Cam) -> SAG")
                self.target_box_name = 'kutu_mavi'
                self.detected_type = 'BLUE'
                self.detection_pub.publish('BLUE')
                self.start_sorting_thread()
            elif cv2.countNonZero(mask_green) > THRESHOLD:
                print(">>> TESPIT: YESIL (Kagit) -> ILERI SOL")
                self.target_box_name = 'kutu_yesil'
                self.detected_type = 'GREEN'
                self.detection_pub.publish('GREEN')
                self.start_sorting_thread()
            elif cv2.countNonZero(mask_gray) > THRESHOLD:
                print(">>> TESPIT: GRI (Metal) -> ILERI SAG")
                self.target_box_name = 'kutu_gri'
                self.detected_type = 'GRAY'
                self.detection_pub.publish('GRAY')
                self.start_sorting_thread()

        except Exception as e:
            rospy.logwarn(f"Kamera hatasi: {e}")

    def draw_box(self, frame, mask, color, text):
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if cv2.contourArea(cnt) > 3000:
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x+w, y+h), color, 3)
                cv2.putText(frame, text, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    def start_sorting_thread(self):
        self.is_busy = True
        self.cycle_start_time = time.time()
        self.trial_no += 1
        self.publish_status("CALISYOR")
        t = threading.Thread(target=self.run_mission)
        t.start()

    def magnet_thread(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.magnet_active:
                try:
                    (trans, rot) = self.tf_listener.lookupTransform(
                        '/base_link', '/wrist_3_link', rospy.Time(0))
                    state = ModelState()
                    state.model_name = self.target_box_name
                    state.reference_frame = 'base_link'
                    state.pose.position.x = trans[0]
                    state.pose.position.y = trans[1]
                    state.pose.position.z = trans[2] - 0.22
                    state.pose.orientation.w = 1.0
                    self.box_service(state)
                except:
                    pass
            rate.sleep()

    def move_joint(self, joints):
        """MoveIt ile joint hedefine git"""
        self.arm.set_joint_value_target(joints)
        plan = self.arm.plan()

        # plan() ROS versiyonuna göre tuple veya direkt plan döndürür
        if isinstance(plan, tuple):
            success, trajectory, _, _ = plan
        else:
            success = True
            trajectory = plan

        if success and len(trajectory.joint_trajectory.points) > 0:
            self.arm.execute(trajectory, wait=True)
            self.arm.stop()
            self.arm.clear_pose_targets()
            return True
        else:
            rospy.logwarn("MoveIt plan bulunamadi, fallback kullaniliyor")
            return False

    def move_fallback(self, joints, duration):
        """MoveIt basarisiz olursa joint trajectory fallback"""
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        pub = rospy.Publisher('/eff_joint_traj_controller/command',
                              JointTrajectory, queue_size=10)
        rospy.sleep(0.5)
        traj = JointTrajectory()
        traj.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        p = JointTrajectoryPoint()
        p.positions = joints
        p.time_from_start = rospy.Duration(duration)
        traj.points.append(p)
        pub.publish(traj)
        rospy.sleep(duration + 0.5)

    def move(self, joints, duration):
        """MoveIt ile hareket et, basarisiz olursa fallback kullan"""
        success = self.move_joint(joints)
        if not success:
            self.move_fallback(joints, duration)

    def run_mission(self):
        grasp_success = False
        try:
            # Yaklas
            self.move([-0.2, -1.57, 1.57, -1.57, -1.57, 0.0], 4.0)
            # Al
            self.move([-0.2, -1.8, 2.1, -1.57, -1.57, 0.0], 4.0)
            rospy.sleep(0.5)

            print(f">>> VAKUM ACILDI: {self.target_box_name} yakalandi.")
            self.magnet_active = True
            rospy.sleep(0.5)
            grasp_success = True

            # Kaldir
            self.move([-0.2, -1.57, 1.57, -1.57, -1.57, 0.0], 3.0)

            # Yonlendir
            if self.detected_type == 'RED':
                print(">>> Hedef: PLASTIK KONTEYNERI (Sol)")
                self.move([1.57, -1.57, 1.57, -1.57, -1.57, 0.0], 5.0)
            elif self.detected_type == 'BLUE':
                print(">>> Hedef: CAM KONTEYNERI (Sag)")
                self.move([-1.57, -1.57, 1.57, -1.57, -1.57, 0.0], 5.0)
            elif self.detected_type == 'GREEN':
                print(">>> Hedef: KAGIT KONTEYNERI (Ileri Sol)")
                self.move([0.8, -1.2, 1.2, -1.57, -1.57, 0.0], 5.0)
            elif self.detected_type == 'GRAY':
                print(">>> Hedef: METAL KONTEYNERI (Ileri Sag)")
                self.move([-0.8, -1.2, 1.2, -1.57, -1.57, 0.0], 5.0)

            # Birak
            self.move_to_drop()

            print(">>> VAKUM KAPATILDI")
            self.magnet_active = False
            rospy.sleep(1.0)

        except Exception as e:
            rospy.logerr(f"Gorev hatasi: {e}")
            self.magnet_active = False

        finally:
            cycle_time = time.time() - self.cycle_start_time if self.cycle_start_time else 0
            log_kpi(self.trial_no, self.detected_type, True, cycle_time, grasp_success)
            print(f">>> KPI | Trial: {self.trial_no} | Tip: {self.detected_type} | Sure: {cycle_time:.2f}s | Basarili: {grasp_success}")

            self.go_home_pose()
            self.is_busy = False
            self.publish_status("HAZIR")
            print(">>> SISTEM HAZIR: Yeni atik bekleniyor...")

    def move_to_drop(self):
        if self.detected_type == 'RED':
            base_angle = 1.57
        elif self.detected_type == 'BLUE':
            base_angle = -1.57
        elif self.detected_type == 'GREEN':
            base_angle = 0.8
        else:
            base_angle = -0.8
        self.move([base_angle, -1.2, 1.5, -1.57, -1.57, 0.0], 3.0)

    def go_home_pose(self):
        self.move([0.0, -1.57, 1.57, -1.57, -1.57, 0.0], 4.0)


if __name__ == '__main__':
    try:
        SortingRobot()
    except rospy.ROSInterruptException:
        moveit_commander.roscpp_shutdown()
