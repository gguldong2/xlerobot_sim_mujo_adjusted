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