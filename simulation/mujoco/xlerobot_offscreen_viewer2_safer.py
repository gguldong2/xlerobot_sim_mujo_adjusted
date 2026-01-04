# 이걸로 바꾸면 xlerobot_offscreen_viewer1.py 대비 “뭐가 더 좋아지냐(기대 효과)""

# 매 프레임 Renderer() 만들었다 닫는 비용이 사라져서
# CPU 사용량이 줄고, 프레임이 더 부드럽고, 입력 반응도 좋아질 가능성이 큼

# 컨텍스트가 가끔 꼬여도 검은 프레임 연속 감지 시 자동으로 복구

# 혹시 나빠질 수 있는 것(주의)

# 네 환경에서 컨텍스트 꼬임이 “정말 자주” 발생하면
# 재사용 중에 검은 프레임이 잠깐 나왔다가 복구될 수 있음
# → 그럴 땐 BLACK_FRAMES_TO_RESET을 2~3으로 낮추거나
# RECREATE_EVERY_N_FRAMES=300 같은 보험을 켜면 안정성이 올라감.

import time
import mujoco
import cv2
import numpy as np
from pynput import keyboard


MJCF_PATH = "scene.xml"

def main():
    m = mujoco.MjModel.from_xml_path(MJCF_PATH)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)


    width, height = 480, 360
    # width, height = 640, 480
    cam_names = [m.camera(i).name for i in range(m.ncam)]
    use_cam = "main" if "main" in cam_names else None
    print("cams:", cam_names, "use:", use_cam)
    print("Press ESC to quit.")
    #####################################################################################
    ###################################추가(키입력)##########################################
    # =========================
    # ✅ Keyboard states (pynput)
    # =========================
    key_states = {
        "home": False, "end": False, "delete": False, "page_down": False,
        "insert": False, "page_up": False,
        "q": False, "a": False, "w": False, "s": False, "e": False, "d": False,
        "u": False, "j": False, "i": False, "k": False, "o": False, "l": False,
    }

    def on_press(key):
        # special keys
        if key == keyboard.Key.home: key_states["home"] = True
        elif key == keyboard.Key.end: key_states["end"] = True
        elif key == keyboard.Key.delete: key_states["delete"] = True
        elif key == keyboard.Key.page_down: key_states["page_down"] = True
        elif key == keyboard.Key.insert: key_states["insert"] = True
        elif key == keyboard.Key.page_up: key_states["page_up"] = True

        # char keys
        try:
            k = key.char.lower()
            if k in key_states:
                key_states[k] = True
        except Exception:
            pass

    def on_release(key):
        if key == keyboard.Key.home: key_states["home"] = False
        elif key == keyboard.Key.end: key_states["end"] = False
        elif key == keyboard.Key.delete: key_states["delete"] = False
        elif key == keyboard.Key.page_down: key_states["page_down"] = False
        elif key == keyboard.Key.insert: key_states["insert"] = False
        elif key == keyboard.Key.page_up: key_states["page_up"] = False

        try:
            k = key.char.lower()
            if k in key_states:
                key_states[k] = False
        except Exception:
            pass

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    
    
    # =========================
    # ✅ Control state (similar to xlerobot_mujoco.py)
    # =========================
    # model.nu: number of actuators
    qCmd = np.zeros(m.nu)     # position-ish command for arms
    qdCmd = np.zeros(m.nu)    # velocity-ish command for chassis
    abs_vel = np.array([6.0, 6.0, 5.0])  # x, y, yaw    #[1.0, 1.0, 1.0]초기값이었음
    kp = 1.0
    
    #####################################################################################
    #####################################################################################

    # ✅ Renderer 한 번 만들고 재사용
    r = mujoco.Renderer(m, width=width, height=height)

    # 로그/복구용
    last_print = 0.0
    black_count = 0

    # ✅ 설정값 (필요하면 조절)
    BLACK_MAX_THRESHOLD = 1      # img.max() <= 1 이면 거의 검정으로 판단
    BLACK_FRAMES_TO_RESET = 5    # 연속 5프레임 검정이면 Renderer 재생성
    RECREATE_EVERY_N_FRAMES = 0  # 0이면 끔. 예: 300이면 300프레임마다 재생성 (보험)
    # TARGET_FPS = 60.0
    TARGET_FPS = 30.0

    frame = 0
    target_dt = 1.0 / TARGET_FPS

    while True:
        t0 = time.time()
        
        ##############################################
        
        # =========================
        # ✅ Update control from key_states
        # =========================
        # (주의) 너의 모델에서 ctrl 인덱스가 아래와 "정확히" 일치한다는 보장은 없어.
        # 하지만 네 기존 xlerobot_mujoco.py와 동일한 가정으로 일단 붙인다:
        # - ctrl[:3]  : chassis (vx, vy, yaw)
        # - ctrl[3:]  : joints/arms

        # 1) chassis desired velocity
        chassis_ref = np.zeros(3)
        if key_states["home"]:
            chassis_ref[0] = abs_vel[0]
        elif key_states["end"]:
            chassis_ref[0] = -abs_vel[0]

        if key_states["delete"]:
            chassis_ref[1] = abs_vel[1]
        elif key_states["page_down"]:
            chassis_ref[1] = -abs_vel[1]

        if key_states["insert"]:
            chassis_ref[2] = abs_vel[2]
        elif key_states["page_up"]:
            chassis_ref[2] = -abs_vel[2]

        qdCmd[0:3] = kp * chassis_ref

        # 2) arm incremental control (same keys as tutorial)
        arm_step = 0.05   #0.005

        # Left arm joints (예: qCmd[3], [4], [5])
        if key_states["q"]:
            qCmd[3] += arm_step
        elif key_states["a"]:
            qCmd[3] -= arm_step

        if key_states["w"]:
            qCmd[4] += arm_step
        elif key_states["s"]:
            qCmd[4] -= arm_step

        if key_states["e"]:
            qCmd[5] += arm_step
        elif key_states["d"]:
            qCmd[5] -= arm_step

        # Right arm joints (예: qCmd[9], [10], [11])
        if key_states["u"]:
            qCmd[9] += arm_step
        elif key_states["j"]:
            qCmd[9] -= arm_step

        if key_states["i"]:
            qCmd[10] += arm_step
        elif key_states["k"]:
            qCmd[10] -= arm_step

        if key_states["o"]:
            qCmd[11] += arm_step
        elif key_states["l"]:
            qCmd[11] -= arm_step

        # 3) apply to mujoco controls (가정: ctrl 구조가 동일)
        if d.ctrl.shape[0] >= 3:
            d.ctrl[:3] = qdCmd[:3]
        if d.ctrl.shape[0] > 3:
            d.ctrl[3:] = qCmd[3:d.ctrl.shape[0]]        
        
        ##############################################
        
        mujoco.mj_step(m, d)

        # 장면 업데이트
        if use_cam:
            r.update_scene(d, camera=use_cam)
        else:
            r.update_scene(d)

        # 렌더 (RGB uint8)
        img = r.render()

        # ✅ 검은 프레임 감지 → 자동 복구
        mn, mx = int(img.min()), int(img.max())
        if mx <= BLACK_MAX_THRESHOLD:
            black_count += 1
        else:
            black_count = 0

        if black_count >= BLACK_FRAMES_TO_RESET:
            # Renderer 재생성으로 컨텍스트 꼬임 복구 시도
            try:
                r.close()
            except Exception:
                pass
            r = mujoco.Renderer(m, width=width, height=height)
            black_count = 0

        # ✅ (선택) 주기적 재생성
        frame += 1
        if RECREATE_EVERY_N_FRAMES and (frame % RECREATE_EVERY_N_FRAMES == 0):
            try:
                r.close()
            except Exception:
                pass
            r = mujoco.Renderer(m, width=width, height=height)

        # 디버그 로그
        if d.time - last_print >= 0.5:
            print("t:", round(d.time, 3), "min/max:", mn, mx, "black_count:", black_count)
            last_print = d.time

        # 표시 (OpenCV)
        bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        cv2.imshow("XLeRobot (offscreen->opencv)", bgr)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            break

        # ✅ FPS 제한 (CPU 과점유 방지 + 일정한 체감)
        elapsed = time.time() - t0
        if elapsed < target_dt:
            time.sleep(target_dt - elapsed)


    try:
        listener.stop()   # ✅ pynput 리스너 종료이게 없으면:프로그램은 종료됐는데 키보드 훅 스레드가 남아서 다음 실행 때 입력 꼬이거나 WSL에서 이상 증상 생길 수 있음
    except Exception:
        pass

    try:
        r.close()
    except Exception:
        pass
    
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
