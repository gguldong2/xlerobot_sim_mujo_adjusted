# # # # """keyboard control for XLeRobot, python mujoco"""

# # # # import time
# # # # import mujoco_viewer
# # # # import numpy as np
# # # # import glfw
# # # # import mujoco


# # # # class XLeRobotController:
# # # #     def __init__(self, mjcf_path):
# # # #         """Initialize the XLeRobot controller with the given MJCF model path."""
# # # #         self.model = mujoco.MjModel.from_xml_path(mjcf_path)
# # # #         self.data = mujoco.MjData(self.model)
# # # #         mujoco.mj_forward(self.model, self.data)

# # # #         self.render_freq = 60  # Hz
# # # #         self.render_interval = 1.0 / self.render_freq
# # # #         self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)

# # # #         self.camera = mujoco.MjvCamera()
# # # #         mujoco.mjv_defaultCamera(self.camera)
# # # #         self.camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
# # # #         self.camera.trackbodyid = self.model.body("chassis").id
# # # #         self.camera.distance = 3.0
# # # #         self.camera.azimuth = 90.0
# # # #         self.camera.elevation = -30.0
# # # #         self.camera.lookat = np.array([0.0, 0.0, 0.0])

# # # #         # self.abs_vel = np.array([1, 1, 1])
# # # #         # self.abs_vel = np.array([5, 5, 3])
# # # #         self.abs_vel = np.array([3, 3, 2])
# # # #         self.chassis_ref_vel = np.zeros(3)
# # # #         self.qCmd = np.zeros(self.model.nu)
# # # #         self.qdCmd = np.zeros(self.model.nu)
# # # #         self.qFb = np.zeros(self.model.nu)
# # # #         self.qdFb = np.zeros(self.model.nu)
# # # #         self.last_render_time = time.time()
# # # #         self.kp = 1

# # # #         self.key_states = {
# # # #             "home": False,  # Forward (+x)
# # # #             "end": False,  # Backward (-x)
# # # #             "delete": False,  # Leftward (+y)
# # # #             "page_down": False,  # Rightward (-y)
# # # #             "insert": False,  # Rotate CCW (+z)
# # # #             "page_up": False,  # Rotate CW (-z)
# # # #             # Left arm controls
# # # #             "q": False,  # Left arm joint 1 positive
# # # #             "a": False,  # Left arm joint 1 negative
# # # #             "w": False,  # Left arm joint 2 positive
# # # #             "s": False,  # Left arm joint 2 negative
# # # #             "e": False,  # Left arm joint 3 positive
# # # #             "d": False,  # Left arm joint 3 negative
# # # #             # Right arm controls
# # # #             "u": False,  # Right arm joint 1 positive
# # # #             "j": False,  # Right arm joint 1 negative
# # # #             "i": False,  # Right arm joint 2 positive
# # # #             "k": False,  # Right arm joint 2 negative
# # # #             "o": False,  # Right arm joint 3 positive
# # # #             "l": False,  # Right arm joint 3 negative
# # # #         }

# # # #     def update_feedback(self):
# # # #         """Calculate current yaw angle from quaternion"""
# # # #         self.qFb = self.data.qpos
# # # #         self.qdFb = self.data.qvel
# # # #     def update_keyboards(self):
# # # #         """숫자 패드와 일반 키 모두 지원하도록 수정"""
# # # #         try:
# # # #             window = self.viewer.window
# # # #             if window is None: return

# # # #             # 일반 키 + 숫자 패드(KP) 키 매핑 추가
# # # #             key_map = {
# # # #                 # [섀시] 일반 키
# # # #                 "home": glfw.KEY_HOME, "end": glfw.KEY_END,
# # # #                 "delete": glfw.KEY_DELETE, "page_down": glfw.KEY_PAGE_DOWN,
# # # #                 "insert": glfw.KEY_INSERT, "page_up": glfw.KEY_PAGE_UP,
                
# # # #                 # [섀시] 숫자 패드 키 (NumLock 꺼진 상태 혹은 켜진 상태 대응)
# # # #                 "home_kp": glfw.KEY_KP_7, "end_kp": glfw.KEY_KP_1,
# # # #                 "delete_kp": glfw.KEY_KP_DECIMAL, "page_down_kp": glfw.KEY_KP_3, # . 키와 3번 키
# # # #                 "insert_kp": glfw.KEY_KP_0, "page_up_kp": glfw.KEY_KP_9,

# # # #                 # [팔]
# # # #                 "q": glfw.KEY_Q, "a": glfw.KEY_A, "w": glfw.KEY_W, "s": glfw.KEY_S,
# # # #                 "e": glfw.KEY_E, "d": glfw.KEY_D,
# # # #                 "u": glfw.KEY_U, "j": glfw.KEY_J, "i": glfw.KEY_I, "k": glfw.KEY_K,
# # # #                 "o": glfw.KEY_O, "l": glfw.KEY_L,
# # # #                 "backspace": glfw.KEY_BACKSPACE
# # # #             }

# # # #             # 키 상태 업데이트 (OR 연산으로 둘 중 하나만 눌려도 True)
# # # #             for name, key in key_map.items():
# # # #                 is_pressed = (glfw.get_key(window, key) == glfw.PRESS)
                
# # # #                 # _kp로 끝나는 키는 원래 이름(home 등)에 합쳐줍니다.
# # # #                 clean_name = name.replace("_kp", "")
# # # #                 if clean_name in self.key_states:
# # # #                     self.key_states[clean_name] = self.key_states[clean_name] or is_pressed
# # # #                 else:
# # # #                     self.key_states[name] = is_pressed
                    
# # # #             # 리셋 로직
# # # #             if self.key_states["backspace"]:
# # # #                 mujoco.mj_resetData(self.model, self.data)
# # # #                 self.qCmd[:] = 0
# # # #                 self.qdCmd[:] = 0
# # # #                 mujoco.mj_forward(self.model, self.data)

# # # #         except Exception:
# # # #             pass


# # # #     # def update_keyboards(self):
# # # #     #     """Check key states using GLFW directly from viewer window"""
# # # #     #     try:
# # # #     #         window = self.viewer.window
# # # #     #         if window is None:
# # # #     #             return

# # # #     #         key_map = {
# # # #     #             "home": glfw.KEY_HOME,
# # # #     #             "end": glfw.KEY_END,
# # # #     #             "delete": glfw.KEY_DELETE,
# # # #     #             "page_down": glfw.KEY_PAGE_DOWN,
# # # #     #             "insert": glfw.KEY_INSERT,
# # # #     #             "page_up": glfw.KEY_PAGE_UP,
# # # #     #             # Left arm keys
# # # #     #             "q": glfw.KEY_Q,
# # # #     #             "a": glfw.KEY_A,
# # # #     #             "w": glfw.KEY_W,
# # # #     #             "s": glfw.KEY_S,
# # # #     #             "e": glfw.KEY_E,
# # # #     #             "d": glfw.KEY_D,
# # # #     #             # Right arm keys
# # # #     #             "u": glfw.KEY_U,
# # # #     #             "j": glfw.KEY_J,
# # # #     #             "i": glfw.KEY_I,
# # # #     #             "k": glfw.KEY_K,
# # # #     #             "o": glfw.KEY_O,
# # # #     #             "l": glfw.KEY_L,
# # # #     #         }

# # # #     #         for key_name, glfw_key in key_map.items():
# # # #     #             self.key_states[key_name] = glfw.get_key(window, glfw_key) == glfw.PRESS

# # # #     #     except Exception:
# # # #     #         pass

# # # #     def update_reference(self):
# # # #         # X-direction (forward/backward)
# # # #         yaw = self.qFb[2]
# # # #         rotmz = np.array(
# # # #             [
# # # #                 [np.cos(yaw), np.sin(yaw), 0],
# # # #                 [-np.sin(yaw), np.cos(yaw), 0],
# # # #                 [0, 0, 1],
# # # #             ]
# # # #         )
# # # #         chassis_vel = rotmz @ self.qdFb[0:3]

# # # #         self.chassis_ref_vel = np.zeros(3)
# # # #         if self.key_states["home"]:
# # # #             self.chassis_ref_vel[0] = self.abs_vel[0]
# # # #         elif self.key_states["end"]:
# # # #             self.chassis_ref_vel[0] = -self.abs_vel[0]
# # # #         if self.key_states["delete"]:
# # # #             self.chassis_ref_vel[1] = self.abs_vel[1]
# # # #         elif self.key_states["page_down"]:
# # # #             self.chassis_ref_vel[1] = -self.abs_vel[1]

# # # #         if self.key_states["insert"]:
# # # #             self.chassis_ref_vel[2] = self.abs_vel[2]
# # # #         elif self.key_states["page_up"]:
# # # #             self.chassis_ref_vel[2] = -self.abs_vel[2]

# # # #         k_p = 10
# # # #         k_p_rot = 100
# # # #         self.qdCmd[0] = self.chassis_ref_vel[0] * np.cos(yaw) + \
# # # #                         self.chassis_ref_vel[1] * np.cos(yaw + 1.5708) + \
# # # #                         k_p * (self.chassis_ref_vel[0] - chassis_vel[0]) * np.cos(yaw) + \
# # # #                         k_p * (self.chassis_ref_vel[1] - chassis_vel[1]) * np.cos(yaw + 1.5708)
# # # #         self.qdCmd[1] = self.chassis_ref_vel[0] * np.sin(yaw) + \
# # # #                         self.chassis_ref_vel[1] * np.sin(yaw + 1.5708) + \
# # # #                         k_p * (self.chassis_ref_vel[0] - chassis_vel[0]) * np.sin(yaw) + \
# # # #                         k_p * (self.chassis_ref_vel[1] - chassis_vel[1]) * np.sin(yaw + 1.5708)
# # # #         self.qdCmd[2] = self.chassis_ref_vel[2] + k_p_rot * (self.chassis_ref_vel[2] - chassis_vel[2])

# # # #         radius = 0.1
# # # #         vel2wheel_matrix = np.array(
# # # #             [[0, 1, -radius], [-np.sqrt(3) * 0.5, -0.5, -radius], [np.sqrt(3) * 0.5, -0.5, -radius]]
# # # #         )
# # # #         coe_vel_to_wheel = 20
# # # #         self.qCmd[15:18] = coe_vel_to_wheel * np.dot(vel2wheel_matrix, chassis_vel)
# # # #         self.qdCmd[2] = np.clip(self.qdCmd[2], -1.0, 1.0)

# # # #         # Left arm joint control (qCmd[3:9])
# # # #         # arm_step = 0.05
# # # #         # arm_step = 0.005
# # # #         arm_step = 0.03

# # # #         # Left arm joint 1
# # # #         if self.key_states["q"]:
# # # #             self.qCmd[3] += arm_step
# # # #         elif self.key_states["a"]:
# # # #             self.qCmd[3] -= arm_step

# # # #         # Left arm joint 2
# # # #         if self.key_states["w"]:
# # # #             self.qCmd[4] += arm_step
# # # #         elif self.key_states["s"]:
# # # #             self.qCmd[4] -= arm_step

# # # #         # Left arm joint 3
# # # #         if self.key_states["e"]:
# # # #             self.qCmd[5] += arm_step
# # # #         elif self.key_states["d"]:
# # # #             self.qCmd[5] -= arm_step

# # # #         # Right arm joint control (qCmd[9:15])
# # # #         # Right arm joint 1
# # # #         if self.key_states["u"]:
# # # #             self.qCmd[9] += arm_step
# # # #         elif self.key_states["j"]:
# # # #             self.qCmd[9] -= arm_step

# # # #         # Right arm joint 2
# # # #         if self.key_states["i"]:
# # # #             self.qCmd[10] += arm_step
# # # #         elif self.key_states["k"]:
# # # #             self.qCmd[10] -= arm_step

# # # #         # Right arm joint 3
# # # #         if self.key_states["o"]:
# # # #             self.qCmd[11] += arm_step
# # # #         elif self.key_states["l"]:
# # # #             self.qCmd[11] -= arm_step

# # # #         # Keep other joints at zero
# # # #         self.qCmd[6:9] = 0.0  # Left arm joints 4-6
# # # #         self.qCmd[12:15] = 0.0  # Right arm joints 4-6
        
# # # #     def update_control(self):
# # # #         """
# # # #         [중요 수정]
# # # #         바퀴 모터가 0, 1, 2번에 있다고 가정하고 제어 값을 넣습니다.
# # # #         만약 로봇 팔이 먼저 정의되어 있다면 이 부분을 수정해야 합니다.
# # # #         """
# # # #         # 팔 제어 (3번 모터부터 끝까지)
# # # #         if self.model.nu > 3:
# # # #             self.data.ctrl[3:] = self.qCmd[3:]

# # # #         # 섀시(바퀴) 제어 - 계산된 qCmd[15:18] 값을 실제 바퀴 모터(0:3)에 넣음
# # # #         # 주의: scene.xml에서 바퀴가 actuators의 맨 앞에 정의되어 있다고 가정
        
# # # #         # 바퀴 속도(qCmd[15:18])를 가져옵니다.
# # # #         wheel_vels = self.qCmd[15:18]
        
# # # #         # 1. 만약 바퀴가 0,1,2번 액추에이터라면:
# # # #         self.data.ctrl[0:3] = wheel_vels
        
# # # #         # 2. (참고) 만약 바퀴가 진짜로 15,16,17번이라면:
# # # #         # self.data.ctrl[15:18] = wheel_vels
        
        
# # # #     # def update_control(self):
# # # #     #     self.qdCmd[0:3] = self.kp * self.qdCmd[0:3]
# # # #     #     self.data.ctrl[:3] = self.qdCmd[:3]
# # # #     #     self.data.ctrl[3:] = self.qCmd[3:]

# # # #     def render_ui(self):
# # # #         current_time = time.time()

# # # #         if current_time - self.last_render_time >= self.render_interval:
# # # #             self.viewer.cam = self.camera
# # # #             self.viewer._overlay[mujoco.mjtGridPos.mjGRID_TOPLEFT] = [
# # # #                 f"Time: {self.data.time:.3f} sec",
# # # #                 "",
# # # #             ]
# # # #             self.viewer._overlay[mujoco.mjtGridPos.mjGRID_BOTTOMRIGHT] = [
# # # #                 "=== CHASSIS MOVEMENT (INCREMENTAL) ===\n"
# # # #                 "Forward/Backward   (+x/-x): press Home/End\n"
# # # #                 "Leftward/Rightward (+y/-y): press Delete/Page Down\n"
# # # #                 "Rotate CCW/CW      (+z/-z): press Insert/Page Up\n"
# # # #                 "\n=== LEFT ARM CONTROLS ===\n"
# # # #                 "Joint1: q(+)/a(-)    Joint2: w(+)/s(-)    Joint3: e(+)/d(-)\n"
# # # #                 "\n=== RIGHT ARM CONTROLS ===\n"
# # # #                 "Joint1: u(+)/j(-)    Joint2: i(+)/k(-)    Joint3: o(+)/l(-)\n"
# # # #                 f"\ncommand: Chassis Vel: [{self.qdCmd[0]:.2f}, {self.qdCmd[1]:.2f}, {self.qdCmd[2]:.2f}]\n"
# # # #                 f"feedback: Chassis Vel: [{self.qdFb[0]:.2f}, {self.qdFb[1]:.2f}, {self.qdFb[2]:.2f}]\n"
# # # #                 f"Left Arm: [{self.qCmd[3]:.2f}, {self.qCmd[4]:.2f}, {self.qCmd[5]:.2f}]\n"
# # # #                 f"Right Arm: [{self.qCmd[9]:.2f}, {self.qCmd[10]:.2f}, {self.qCmd[11]:.2f}]",
# # # #                 "",
# # # #             ]

# # # #             self.viewer.render()
# # # #             self.last_render_time = current_time

# # # #     def run(self):
# # # #         """Main control loop for XLeRobot keyboard control."""
# # # #         print("Starting XLeRobot keyboard Controller...")

# # # #         while self.viewer.is_alive:
# # # #             self.update_feedback()
# # # #             self.update_keyboards()
# # # #             self.update_reference()
# # # #             self.update_control()
# # # #             mujoco.mj_step(self.model, self.data)
# # # #             self.render_ui()
# # # #             time.sleep(0.002)

# # # #         self.cleanup()

# # # #     def cleanup(self):
# # # #         self.viewer.close()
# # # #         print("XLeRobot controller stopped.")


# # # # def main():
# # # #     try:
# # # #         mjcf_path = "scene.xml"
# # # #         controller = XLeRobotController(mjcf_path)
# # # #         controller.run()
# # # #     except KeyboardInterrupt:
# # # #         print("\nReceived keyboard interrupt, shutting down...")


# # # # if __name__ == "__main__":
# # # #     main()
# # # """keyboard control for XLeRobot (Diagnostics Mode)"""

# # # import time
# # # import mujoco_viewer
# # # import numpy as np
# # # import glfw
# # # import mujoco

# # # class XLeRobotController:
# # #     def __init__(self, mjcf_path):
# # #         self.model = mujoco.MjModel.from_xml_path(mjcf_path)
# # #         self.data = mujoco.MjData(self.model)
# # #         mujoco.mj_forward(self.model, self.data)

# # #         self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)
        
# # #         # === [핵심 1] 유령 브레이크(0,1,2번) 완전 제거 ===
# # #         # gain(힘) 뿐만 아니라 force range(허용 힘 범위)를 0으로 만들어버림
# # #         # 이제 이 모터들은 물리적으로 힘을 낼 수가 없습니다.
# # #         self.model.actuator_gainprm[0:3, 0] = 0
# # #         self.model.actuator_forcerange[0:3, :] = 0
# # #         print(">>> Ghost Actuators (0-2) fully disabled.")

# # #         # === [핵심 2] 바퀴 모터(15,16,17번) 파워 100배 강화 ===
# # #         # 기존 kv=10 -> kv=1000으로 변경
# # #         self.model.actuator_gainprm[15:18, 0] = 1000.0
        
# # #         # === [핵심 3] 마찰력 강화 (미끄러짐 방지) ===
# # #         for i in range(self.model.ngeom):
# # #             self.model.geom_friction[i, 0] = 2.0 

# # #         self.abs_vel = np.array([5.0, 5.0, 4.0]) 
# # #         self.chassis_ref_vel = np.zeros(3)
# # #         self.qCmd = np.zeros(self.model.nu)
# # #         self.key_states = {"home":False, "end":False, "delete":False, "page_down":False, "insert":False, "page_up":False, "backspace":False}

# # #         # 카메라 설정
# # #         self.camera = mujoco.MjvCamera()
# # #         mujoco.mjv_defaultCamera(self.camera)
# # #         self.camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
# # #         self.camera.trackbodyid = self.model.body("chassis").id
# # #         self.camera.distance = 3.0
# # #         self.camera.lookat = np.array([0.0, 0.0, 0.0])

# # #     def update_keyboards(self):
# # #         try:
# # #             window = self.viewer.window
# # #             if window is None: return
            
# # #             # 키 매핑 (NumPad 포함)
# # #             key_map = {
# # #                 "home": [glfw.KEY_HOME, glfw.KEY_KP_7], "end": [glfw.KEY_END, glfw.KEY_KP_1],
# # #                 "delete": [glfw.KEY_DELETE, glfw.KEY_KP_DECIMAL], "page_down": [glfw.KEY_PAGE_DOWN, glfw.KEY_KP_3],
# # #                 "insert": [glfw.KEY_INSERT, glfw.KEY_KP_0], "page_up": [glfw.KEY_PAGE_UP, glfw.KEY_KP_9],
# # #                 "backspace": [glfw.KEY_BACKSPACE, None]
# # #             }

# # #             for name, keys in key_map.items():
# # #                 pressed = False
# # #                 for k in keys:
# # #                     if k and glfw.get_key(window, k) == glfw.PRESS:
# # #                         pressed = True
# # #                 self.key_states[name] = pressed
            
# # #             if self.key_states["backspace"]:
# # #                 mujoco.mj_resetData(self.model, self.data)
# # #                 self.qCmd[:] = 0
# # #                 mujoco.mj_forward(self.model, self.data)

# # #         except Exception: pass

# # #     def update_control(self):
# # #         # 입력 -> 목표 속도
# # #         self.chassis_ref_vel[:] = 0
# # #         if self.key_states["home"]: self.chassis_ref_vel[0] = self.abs_vel[0]
# # #         elif self.key_states["end"]: self.chassis_ref_vel[0] = -self.abs_vel[0]
# # #         if self.key_states["delete"]: self.chassis_ref_vel[1] = self.abs_vel[1]
# # #         elif self.key_states["page_down"]: self.chassis_ref_vel[1] = -self.abs_vel[1]
# # #         if self.key_states["insert"]: self.chassis_ref_vel[2] = self.abs_vel[2]
# # #         elif self.key_states["page_up"]: self.chassis_ref_vel[2] = -self.abs_vel[2]

# # #         # 옴니휠 계산
# # #         radius = 0.1
# # #         vel2wheel = np.array([[0, 1, -radius], [-0.866, -0.5, -radius], [0.866, -0.5, -radius]])
# # #         # 속도 비율을 50으로 높임 (모터가 더 빨리 돌도록)
# # #         wheel_vels = 50.0 * np.dot(vel2wheel, self.chassis_ref_vel)
        
# # #         # 명령 주입
# # #         self.data.ctrl[0:3] = 0 # 유령 브레이크 해제
# # #         self.data.ctrl[15:18] = wheel_vels

# # #         # === [진단 로그 출력] ===
# # #         # Home 키를 눌렀을 때만 출력
# # #         if self.key_states["home"]:
# # #             # 1. 명령값(Input) vs 2. 실제 바퀴속도(Wheel RPM) vs 3. 로봇 실제속도(Body Vel)
# # #             print(f"CMD: {wheel_vels[0]:.1f} | Wheel Vel: {self.data.qvel[15]:.1f} | Robot X-Vel: {self.data.qvel[0]:.4f}")

# # #     def run(self):
# # #         print(">>> Controller Started. Press HOME key and check the terminal numbers!")
# # #         while self.viewer.is_alive:
# # #             self.update_keyboards()
# # #             self.update_control()
# # #             mujoco.mj_step(self.model, self.data)
# # #             self.viewer.render()
# # #         self.viewer.close()

# # # if __name__ == "__main__":
# # #     XLeRobotController("scene.xml").run()


# # """
# # XLeRobot Ultimate Controller
# # - Features: Arm Control, Omni-Wheel Drive, Z-Height Adjustment, Physics Override
# # """

# # import time
# # import mujoco_viewer
# # import numpy as np
# # import glfw
# # import mujoco

# # class XLeRobotController:
# #     def __init__(self, mjcf_path):
# #         self.model = mujoco.MjModel.from_xml_path(mjcf_path)
# #         self.data = mujoco.MjData(self.model)
# #         mujoco.mj_forward(self.model, self.data)

# #         self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)
        
# #         # === [물리 강제 수정] ===
# #         # 1. 유령 브레이크(0,1,2번) 완전 제거
# #         self.model.actuator_gainprm[0:3, 0] = 0
# #         self.model.actuator_forcerange[0:3, :] = 0
        
# #         # 2. 바퀴 모터(15,16,17번) 파워 강화
# #         self.model.actuator_gainprm[15:18, 0] = 500.0
        
# #         # 3. 마찰력 강화
# #         for i in range(self.model.ngeom):
# #             self.model.geom_friction[i, 0] = 1.0 

# #         # 설정
# #         self.abs_vel = np.array([4.0, 4.0, 3.0]) 
# #         self.chassis_ref_vel = np.zeros(3)
# #         self.qCmd = np.zeros(self.model.nu)
        
# #         # 초기 높이 저장
# #         self.base_z_idx = self.model.body("chassis").id
# #         self.current_z_offset = self.model.body_pos[self.base_z_idx, 2]
# #         print(f">>> Initial Height (Z): {self.current_z_offset}")

# #         self.key_states = {}
        
# #         self.camera = mujoco.MjvCamera()
# #         mujoco.mjv_defaultCamera(self.camera)
# #         self.camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
# #         self.camera.trackbodyid = self.model.body("chassis").id
# #         self.camera.distance = 3.5
# #         self.camera.elevation = -40.0

# #     def update_keyboards(self):
# #         window = self.viewer.window
# #         if window is None: return
        
# #         # 키 매핑
# #         keys = {
# #             "home": [glfw.KEY_HOME, glfw.KEY_KP_7], "end": [glfw.KEY_END, glfw.KEY_KP_1],
# #             "delete": [glfw.KEY_DELETE, glfw.KEY_KP_DECIMAL], "page_down": [glfw.KEY_PAGE_DOWN, glfw.KEY_KP_3],
# #             "insert": [glfw.KEY_INSERT, glfw.KEY_KP_0], "page_up": [glfw.KEY_PAGE_UP, glfw.KEY_KP_9],
# #             "q": [glfw.KEY_Q], "a": [glfw.KEY_A], "w": [glfw.KEY_W], "s": [glfw.KEY_S], "e": [glfw.KEY_E], "d": [glfw.KEY_D],
# #             "u": [glfw.KEY_U], "j": [glfw.KEY_J], "i": [glfw.KEY_I], "k": [glfw.KEY_K], "o": [glfw.KEY_O], "l": [glfw.KEY_L],
# #             "up_z": [glfw.KEY_RIGHT_BRACKET], "down_z": [glfw.KEY_LEFT_BRACKET], # [, ] 키로 높이 조절
# #             "backspace": [glfw.KEY_BACKSPACE]
# #         }

# #         self.key_states = {}
# #         for name, key_list in keys.items():
# #             pressed = False
# #             for k in key_list:
# #                 if glfw.get_key(window, k) == glfw.PRESS:
# #                     pressed = True
# #             self.key_states[name] = pressed

# #         # 리셋
# #         if self.key_states["backspace"]:
# #             mujoco.mj_resetData(self.model, self.data)
# #             self.qCmd[:] = 0
# #             mujoco.mj_forward(self.model, self.data)

# #         # 높이 조절 (실시간)
# #         if self.key_states["up_z"]:
# #             self.current_z_offset += 0.0005
# #             self.model.body_pos[self.base_z_idx, 2] = self.current_z_offset
# #         elif self.key_states["down_z"]:
# #             self.current_z_offset -= 0.0005
# #             self.model.body_pos[self.base_z_idx, 2] = self.current_z_offset

# #     def update_control(self):
# #         # 1. 섀시 속도 계산
# #         self.chassis_ref_vel[:] = 0
# #         if self.key_states["home"]: self.chassis_ref_vel[0] = self.abs_vel[0]
# #         elif self.key_states["end"]: self.chassis_ref_vel[0] = -self.abs_vel[0]
# #         if self.key_states["delete"]: self.chassis_ref_vel[1] = self.abs_vel[1]
# #         elif self.key_states["page_down"]: self.chassis_ref_vel[1] = -self.abs_vel[1]
# #         if self.key_states["insert"]: self.chassis_ref_vel[2] = self.abs_vel[2]
# #         elif self.key_states["page_up"]: self.chassis_ref_vel[2] = -self.abs_vel[2]

# #         # 옴니휠 기구학
# #         radius = 0.1
# #         vel2wheel = np.array([[0, 1, -radius], [-0.866, -0.5, -radius], [0.866, -0.5, -radius]])
# #         wheel_vels = 20.0 * np.dot(vel2wheel, self.chassis_ref_vel)
        
# #         self.qCmd[15:18] = wheel_vels

# #         # 2. 팔 제어 (복구됨!)
# #         step = 0.02
# #         # 왼팔(9~14)
# #         if self.key_states["q"]: self.qCmd[9] += step
# #         elif self.key_states["a"]: self.qCmd[9] -= step
# #         if self.key_states["w"]: self.qCmd[10] += step
# #         elif self.key_states["s"]: self.qCmd[10] -= step
# #         if self.key_states["e"]: self.qCmd[11] += step
# #         elif self.key_states["d"]: self.qCmd[11] -= step
# #         # 오른팔(3~8)
# #         if self.key_states["u"]: self.qCmd[3] += step
# #         elif self.key_states["j"]: self.qCmd[3] -= step
# #         if self.key_states["i"]: self.qCmd[4] += step
# #         elif self.key_states["k"]: self.qCmd[4] -= step
# #         if self.key_states["o"]: self.qCmd[5] += step
# #         elif self.key_states["l"]: self.qCmd[5] -= step

# #         # 3. 명령 주입
# #         self.data.ctrl[0:3] = 0 # 유령 브레이크 해제
# #         self.data.ctrl[3:15] = self.qCmd[3:15] # 팔
# #         self.data.ctrl[15:18] = self.qCmd[15:18] # 바퀴

# #     def render_ui(self):
# #         # 화면 표시
# #         w1, w2, w3 = self.data.qvel[15], self.data.qvel[16], self.data.qvel[17]
        
# #         status = (
# #             f"Height(Z): {self.current_z_offset:.4f} (Use [ / ] to adjust)\n"
# #             f"Wheel Vel: {w1:.1f} | {w2:.1f} | {w3:.1f}\n"
# #             f"Arm Key : {'ACTIVE' if self.key_states['q'] else 'IDLE'}"
# #         )
        
# #         self.viewer._overlay[mujoco.mjtGridPos.mjGRID_BOTTOMRIGHT] = ["STATUS", status]
# #         self.viewer.render()

# #     def run(self):
# #         print(">>> XLeRobot Controller Started.")
# #         print(">>> Use [ ] keys to adjust height if stuck!")
        
# #         while self.viewer.is_alive:
# #             self.update_keyboards()
# #             self.update_control()
# #             mujoco.mj_step(self.model, self.data)
# #             self.render_ui()
# #             time.sleep(0.002)
# #         self.viewer.close()

# # if __name__ == "__main__":
# #     # 파일 경로가 맞는지 확인하세요
# #     XLeRobotController("scene.xml").run()


# """
# XLeRobot Controller: High Traction Version
# - Fixes: Wheel slip, Floating, Friction, Damping
# """

# import time
# import mujoco_viewer
# import numpy as np
# import glfw
# import mujoco

# class XLeRobotController:
#     def __init__(self, mjcf_path):
#         self.model = mujoco.MjModel.from_xml_path(mjcf_path)
#         self.data = mujoco.MjData(self.model)
#         mujoco.mj_forward(self.model, self.data)

#         self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)
        
#         # === [설정 1] 시각화: 바닥에 닿는지 확인하기 위해 '빨간 점' 켜기 ===
#         # 실행 후 바퀴 밑에 빨간 점들이 보여야 정상입니다.
#         self.viewer.vopt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = 1 
#         self.viewer.vopt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = 1

#         # =================================================================
#         # 🔥🔥🔥 [물리 엔진 해킹] 접지력(Traction) 강제 주입 🔥🔥🔥
#         # =================================================================
        
#         # 1. 유령 브레이크(가짜 관절)의 저항(Damping)을 0으로 제거
#         # (이게 0.1로 남아있으면 끈적한 물엿 위를 걷는 것과 같습니다)
#         self.model.dof_damping[0] = 0
#         self.model.dof_damping[1] = 0
#         self.model.dof_damping[2] = 0
        
#         # 2. 유령 모터 힘 끄기
#         self.model.actuator_gainprm[0:3, 0] = 0
#         self.model.actuator_forcerange[0:3, :] = 0
        
#         # 3. 바퀴 모터 파워 초강력 부스트 (5000배)
#         # 헛바퀴가 돌 정도면 힘은 충분하지만, 혹시 몰라 더 올립니다.
#         self.model.actuator_gainprm[15:18, 0] = 2000.0
        
#         # 4. [핵심] 마찰력(Friction) 100배 강화 (Super Sticky Tires)
#         # 얼음판(0.001)을 고무타이어(2.0)로 바꿉니다.
#         for i in range(self.model.ngeom):
#             self.model.geom_friction[i, 0] = 2.0  # 미끄러짐 마찰
#             self.model.geom_friction[i, 1] = 0.1  # 비틀림 마찰
#             self.model.geom_friction[i, 2] = 0.1  # 구름 마찰
#             # 바닥을 좀 더 단단하게 만듭니다 (solref)
#             self.model.geom_solref[i, :] = np.array([0.004, 1.0]) 

#         # 5. 로봇 무게 중심 살짝 내리기 (안정성)
#         self.base_z_idx = self.model.body("chassis").id
        
#         # =================================================================

#         self.abs_vel = np.array([5.0, 5.0, 3.0]) 
#         self.chassis_ref_vel = np.zeros(3)
#         self.qCmd = np.zeros(self.model.nu)
        
#         # 초기 Z값 미세 조정 (바닥에 꾹 눌리도록)
#         self.current_z_offset = 0.025 # 0.035는 너무 떴을 수 있음
#         self.model.body_pos[self.base_z_idx, 2] = self.current_z_offset
        
#         self.key_states = {}
        
#         self.camera = mujoco.MjvCamera()
#         mujoco.mjv_defaultCamera(self.camera)
#         self.camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
#         self.camera.trackbodyid = self.model.body("chassis").id
#         self.camera.distance = 3.5
#         self.camera.elevation = -40.0

#     def update_keyboards(self):
#         window = self.viewer.window
#         if window is None: return
        
#         keys = {
#             "home": [glfw.KEY_HOME, glfw.KEY_KP_7], "end": [glfw.KEY_END, glfw.KEY_KP_1],
#             "delete": [glfw.KEY_DELETE, glfw.KEY_KP_DECIMAL], "page_down": [glfw.KEY_PAGE_DOWN, glfw.KEY_KP_3],
#             "insert": [glfw.KEY_INSERT, glfw.KEY_KP_0], "page_up": [glfw.KEY_PAGE_UP, glfw.KEY_KP_9],
#             "q": [glfw.KEY_Q], "a": [glfw.KEY_A], "w": [glfw.KEY_W], "s": [glfw.KEY_S], "e": [glfw.KEY_E], "d": [glfw.KEY_D],
#             "u": [glfw.KEY_U], "j": [glfw.KEY_J], "i": [glfw.KEY_I], "k": [glfw.KEY_K], "o": [glfw.KEY_O], "l": [glfw.KEY_L],
#             "up_z": [glfw.KEY_RIGHT_BRACKET], "down_z": [glfw.KEY_LEFT_BRACKET],
#             "backspace": [glfw.KEY_BACKSPACE]
#         }

#         self.key_states = {}
#         for name, key_list in keys.items():
#             pressed = False
#             for k in key_list:
#                 if glfw.get_key(window, k) == glfw.PRESS:
#                     pressed = True
#             self.key_states[name] = pressed

#         if self.key_states["backspace"]:
#             mujoco.mj_resetData(self.model, self.data)
#             self.qCmd[:] = 0
#             mujoco.mj_forward(self.model, self.data)

#         # 높이 실시간 조절 (매우 중요)
#         if self.key_states["up_z"]:
#             self.current_z_offset += 0.0002
#             self.model.body_pos[self.base_z_idx, 2] = self.current_z_offset
#         elif self.key_states["down_z"]:
#             self.current_z_offset -= 0.0002
#             self.model.body_pos[self.base_z_idx, 2] = self.current_z_offset

#     def update_control(self):
#         # 1. 섀시 속도
#         self.chassis_ref_vel[:] = 0
#         if self.key_states["home"]: self.chassis_ref_vel[0] = self.abs_vel[0]
#         elif self.key_states["end"]: self.chassis_ref_vel[0] = -self.abs_vel[0]
#         if self.key_states["delete"]: self.chassis_ref_vel[1] = self.abs_vel[1]
#         elif self.key_states["page_down"]: self.chassis_ref_vel[1] = -self.abs_vel[1]
#         if self.key_states["insert"]: self.chassis_ref_vel[2] = self.abs_vel[2]
#         elif self.key_states["page_up"]: self.chassis_ref_vel[2] = -self.abs_vel[2]

#         # 옴니휠 기구학
#         radius = 0.1
#         vel2wheel = np.array([[0, 1, -radius], [-0.866, -0.5, -radius], [0.866, -0.5, -radius]])
        
#         # [중요] 모터 속도(RPM)를 훨씬 더 높임 (50.0 -> 100.0)
#         wheel_vels = 100.0 * np.dot(vel2wheel, self.chassis_ref_vel)
#         self.qCmd[15:18] = wheel_vels

#         # 2. 팔 제어
#         step = 0.02
#         if self.key_states["q"]: self.qCmd[9] += step
#         elif self.key_states["a"]: self.qCmd[9] -= step
#         if self.key_states["w"]: self.qCmd[10] += step
#         elif self.key_states["s"]: self.qCmd[10] -= step
#         if self.key_states["e"]: self.qCmd[11] += step
#         elif self.key_states["d"]: self.qCmd[11] -= step
#         if self.key_states["u"]: self.qCmd[3] += step
#         elif self.key_states["j"]: self.qCmd[3] -= step
#         if self.key_states["i"]: self.qCmd[4] += step
#         elif self.key_states["k"]: self.qCmd[4] -= step
#         if self.key_states["o"]: self.qCmd[5] += step
#         elif self.key_states["l"]: self.qCmd[5] -= step

#         # 3. 명령 주입
#         self.data.ctrl[0:3] = 0 
#         self.data.ctrl[3:15] = self.qCmd[3:15] 
#         self.data.ctrl[15:18] = self.qCmd[15:18] 

#     def render_ui(self):
#         w1, w2, w3 = self.data.qvel[15], self.data.qvel[16], self.data.qvel[17]
#         # 로봇의 실제 이동 속도 (X, Y)
#         vx, vy = self.data.qvel[0], self.data.qvel[1]
        
#         status = (
#             f"Height(Z): {self.current_z_offset:.4f} (Use [ / ] to fix)\n"
#             f"Robot Vel: X={vx:.3f}, Y={vy:.3f}\n"
#             f"Wheel RPM: {w1:.0f} | {w2:.0f} | {w3:.0f}"
#         )
        
#         self.viewer._overlay[mujoco.mjtGridPos.mjGRID_BOTTOMRIGHT] = ["STATUS", status]
#         self.viewer.render()

#     def run(self):
#         print(">>> Traction Control Enabled.")
#         print(">>> If wheels spin red but no move -> Press ']' key slowly to lower robot!")
        
#         while self.viewer.is_alive:
#             self.update_keyboards()
#             self.update_control()
#             mujoco.mj_step(self.model, self.data)
#             self.render_ui()
#             time.sleep(0.002)
#         self.viewer.close()

# if __name__ == "__main__":
#     XLeRobotController("scene.xml").run()


"""
XLeRobot Controller: Smooth Driving Version
- Visuals: Clean (No debug arrows)
- Physics: Tuned for stable driving (Good grip, manageable speed)
"""

import time
import mujoco_viewer
import numpy as np
import glfw
import mujoco

class XLeRobotController:
    def __init__(self, mjcf_path):
        self.model = mujoco.MjModel.from_xml_path(mjcf_path)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)

        self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)
        
        # === [수정 1] 화면 깨끗하게 (막대기 제거) ===
        self.viewer.vopt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = 0
        self.viewer.vopt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = 0 

        # === [수정 2] 물리 엔진 튜닝 (부드러운 주행) ===
        
        # 1. 유령 브레이크 제거 (유지)
        self.model.dof_damping[0:3] = 0
        self.model.actuator_gainprm[0:3, 0] = 0
        self.model.actuator_forcerange[0:3, :] = 0
        
        # 2. 바퀴 모터 파워 조절 (2000 -> 200)
        # 너무 강하면 제어가 안 되므로 적당히 줄입니다.
        self.model.actuator_gainprm[15:18, 0] = 200.0
        
        # 3. 마찰력 유지 (미끄러짐 방지)
        for i in range(self.model.ngeom):
            self.model.geom_friction[i, 0] = 1.0 
            self.model.geom_friction[i, 1] = 0.1
            self.model.geom_friction[i, 2] = 0.1

        # 4. 로봇 높이 설정 (바닥에 딱 붙게)
        self.base_z_idx = self.model.body("chassis").id
        self.current_z_offset = 0.024 # 바닥에 안정적으로 안착하는 높이
        self.model.body_pos[self.base_z_idx, 2] = self.current_z_offset
        
        # === [수정 3] 최고 속도 제한 ===
        # [전진속도, 좌우속도, 회전속도]
        self.abs_vel = np.array([1.5, 1.5, 1.5]) 
        
        self.chassis_ref_vel = np.zeros(3)
        self.qCmd = np.zeros(self.model.nu)
        self.key_states = {}
        
        # 카메라
        self.camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(self.camera)
        self.camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        self.camera.trackbodyid = self.model.body("chassis").id
        self.camera.distance = 3.0
        self.camera.elevation = -30.0

    def update_keyboards(self):
        window = self.viewer.window
        if window is None: return
        
        keys = {
            "home": [glfw.KEY_HOME, glfw.KEY_KP_7], "end": [glfw.KEY_END, glfw.KEY_KP_1],
            "delete": [glfw.KEY_DELETE, glfw.KEY_KP_DECIMAL], "page_down": [glfw.KEY_PAGE_DOWN, glfw.KEY_KP_3],
            "insert": [glfw.KEY_INSERT, glfw.KEY_KP_0], "page_up": [glfw.KEY_PAGE_UP, glfw.KEY_KP_9],
            "q": [glfw.KEY_Q], "a": [glfw.KEY_A], "w": [glfw.KEY_W], "s": [glfw.KEY_S], "e": [glfw.KEY_E], "d": [glfw.KEY_D],
            "u": [glfw.KEY_U], "j": [glfw.KEY_J], "i": [glfw.KEY_I], "k": [glfw.KEY_K], "o": [glfw.KEY_O], "l": [glfw.KEY_L],
            "up_z": [glfw.KEY_RIGHT_BRACKET], "down_z": [glfw.KEY_LEFT_BRACKET],
            "backspace": [glfw.KEY_BACKSPACE]
        }

        self.key_states = {}
        for name, key_list in keys.items():
            pressed = False
            for k in key_list:
                if glfw.get_key(window, k) == glfw.PRESS:
                    pressed = True
            self.key_states[name] = pressed

        if self.key_states["backspace"]:
            mujoco.mj_resetData(self.model, self.data)
            self.qCmd[:] = 0
            mujoco.mj_forward(self.model, self.data)
            self.model.body_pos[self.base_z_idx, 2] = self.current_z_offset # 리셋 시 높이 복구

        if self.key_states["up_z"]:
            self.current_z_offset += 0.0002
            self.model.body_pos[self.base_z_idx, 2] = self.current_z_offset
        elif self.key_states["down_z"]:
            self.current_z_offset -= 0.0002
            self.model.body_pos[self.base_z_idx, 2] = self.current_z_offset

    def update_control(self):
        # 1. 섀시 입력
        self.chassis_ref_vel[:] = 0
        if self.key_states["home"]: self.chassis_ref_vel[0] = self.abs_vel[0]
        elif self.key_states["end"]: self.chassis_ref_vel[0] = -self.abs_vel[0]
        if self.key_states["delete"]: self.chassis_ref_vel[1] = self.abs_vel[1]
        elif self.key_states["page_down"]: self.chassis_ref_vel[1] = -self.abs_vel[1]
        if self.key_states["insert"]: self.chassis_ref_vel[2] = self.abs_vel[2]
        elif self.key_states["page_up"]: self.chassis_ref_vel[2] = -self.abs_vel[2]

        # 2. 옴니휠 계산
        radius = 0.1
        vel2wheel = np.array([[0, 1, -radius], [-0.866, -0.5, -radius], [0.866, -0.5, -radius]])
        
        # [수정] 속도 배율을 100에서 30으로 줄임 (부드럽게)
        wheel_vels = 30.0 * np.dot(vel2wheel, self.chassis_ref_vel)
        self.qCmd[15:18] = wheel_vels

        # 3. 팔 제어
        step = 0.02
        if self.key_states["q"]: self.qCmd[9] += step
        elif self.key_states["a"]: self.qCmd[9] -= step
        if self.key_states["w"]: self.qCmd[10] += step
        elif self.key_states["s"]: self.qCmd[10] -= step
        if self.key_states["e"]: self.qCmd[11] += step
        elif self.key_states["d"]: self.qCmd[11] -= step
        if self.key_states["u"]: self.qCmd[3] += step
        elif self.key_states["j"]: self.qCmd[3] -= step
        if self.key_states["i"]: self.qCmd[4] += step
        elif self.key_states["k"]: self.qCmd[4] -= step
        if self.key_states["o"]: self.qCmd[5] += step
        elif self.key_states["l"]: self.qCmd[5] -= step

        # 4. 명령 주입
        self.data.ctrl[0:3] = 0 
        self.data.ctrl[3:15] = self.qCmd[3:15] 
        self.data.ctrl[15:18] = self.qCmd[15:18] 

    # def render_ui(self):
    #     vx, vy = self.data.qvel[0], self.data.qvel[1]
    #     w1, w2, w3 = self.data.qvel[15], self.data.qvel[16], self.data.qvel[17]
        
    #     status = (
    #         f"Vel(X,Y): {vx:.2f}, {vy:.2f}\n"
    #         f"Wheel RPM: {w1:.0f}|{w2:.0f}|{w3:.0f}\n"
    #         "Normal Mode: Clean Screen"
    #     )
        
    #     self.viewer._overlay[mujoco.mjtGridPos.mjGRID_BOTTOMRIGHT] = ["STATUS", status]
    #     self.viewer.render()
    def render_ui(self):
            # =========================================================
            # 🛡️ [철통 방어] 모든 디버그 단축키 무력화 (안전 버전)
            # 에러가 나지 않는 항목만 골라서 끕니다.
            # =========================================================
            
            # 1. 가장 문제되는 '긴 막대기(힘)'와 '화살표(관절)' 끄기 (이건 확실히 존재함)
            self.viewer.vopt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = 0
            self.viewer.vopt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = 0
            self.viewer.vopt.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = 0
            self.viewer.vopt.flags[mujoco.mjtVisFlag.mjVIS_ACTUATOR] = 0
            self.viewer.vopt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = 0

            # 2. 버전마다 이름이 다를 수 있는 것들은 있으면 끄고, 없으면 무시(pass)
            # (방금 에러 났던 mjVIS_FRAME 등 처리)
            debug_flags = [
                "mjVIS_FRAME", "mjVIS_COM", "mjVIS_CONSTRAINT", 
                "mjVIS_SKELETON", "mjVIS_SCRATCH", "mjVIS_SITE"
            ]
            
            for flag_name in debug_flags:
                try:
                    # 라이브러리에 해당 플래그가 있는지 확인하고 끄기
                    if hasattr(mujoco.mjtVisFlag, flag_name):
                        flag_id = getattr(mujoco.mjtVisFlag, flag_name)
                        self.viewer.vopt.flags[flag_id] = 0
                except Exception:
                    pass # 없으면 그냥 넘어감 (에러 방지)

            # =========================================================

            # 기존 UI 그리기
            vx, vy = self.data.qvel[0], self.data.qvel[1]
            w1, w2, w3 = self.data.qvel[15], self.data.qvel[16], self.data.qvel[17]
            
            status = (
                f"Vel(X,Y): {vx:.2f}, {vy:.2f}\n"
                f"Wheel RPM: {w1:.0f}|{w2:.0f}|{w3:.0f}\n"
                "Visuals: Clean Mode"
            )
            
            self.viewer._overlay[mujoco.mjtGridPos.mjGRID_BOTTOMRIGHT] = ["STATUS", status]
            self.viewer.render()

    def run(self):
        print(">>> XLeRobot Smooth Control Started.")
        print(">>> Visual clutter removed. Physics tuned.")
        
        while self.viewer.is_alive:
            self.update_keyboards()
            self.update_control()
            mujoco.mj_step(self.model, self.data)
            self.render_ui()
            time.sleep(0.002)
        self.viewer.close()

if __name__ == "__main__":
    XLeRobotController("scene.xml").run()