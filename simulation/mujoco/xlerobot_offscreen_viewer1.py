# import time
# import mujoco
# import cv2
# import numpy as np

# MJCF_PATH = "scene.xml"

# def main():
#     m = mujoco.MjModel.from_xml_path(MJCF_PATH)
#     d = mujoco.MjData(m)
#     mujoco.mj_forward(m, d)

#     # offscreen renderer (WSL에서도 안정적)
#     width, height = 640, 480
#     r = mujoco.Renderer(m, width=width, height=height)

#     # 카메라를 "main"으로 쓰고 싶으면 scene.xml에 camera name="main"이 있어야 함
#     # 없으면 기본 카메라로 렌더됨
#     camera_name = "main" if "main" in [m.camera(i).name for i in range(m.ncam)] else None

#     print("Offscreen viewer running. Press ESC to quit.")
#     while True:
#         mujoco.mj_step(m, d)

#         if camera_name:
#             r.update_scene(d, camera="main")
#         else:
#             r.update_scene(d)

#         img = r.render()  # RGB uint8
        
#         # ✅ 여기에 넣기 (img가 만들어진 직후)
#         if int(d.time * 1000) % 500 == 0:
#             print("t:", round(d.time, 3), "min/max:", int(img.min()), int(img.max()))
    
#         bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
#         cv2.imshow("XLeRobot (offscreen->opencv)", bgr)

#         key = cv2.waitKey(1) & 0xFF
#         if key == 27:  # ESC
#             break

#         time.sleep(0.002)

#     cv2.destroyAllWindows()

# if __name__ == "__main__":
#     main()


import time
import mujoco
import cv2

MJCF_PATH = "scene.xml"

def main():
    m = mujoco.MjModel.from_xml_path(MJCF_PATH)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)

    width, height = 640, 480
    cam_names = [m.camera(i).name for i in range(m.ncam)]
    use_cam = "main" if "main" in cam_names else None
    print("cams:", cam_names, "use:", use_cam)
    print("Press ESC to quit.")

    last_print = 0.0
    while True:
        mujoco.mj_step(m, d)

        # ✅ Renderer를 매 loop 생성 (WSL에서 컨텍스트 꼬임 우회)
        r = mujoco.Renderer(m, width=width, height=height)
        if use_cam:
            r.update_scene(d, camera=use_cam)
        else:
            r.update_scene(d)
        img = r.render()
        r.close()

        if d.time - last_print >= 0.5:
            print("t:", round(d.time, 3), "min/max:", int(img.min()), int(img.max()))
            last_print = d.time

        bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        cv2.imshow("XLeRobot (offscreen->opencv)", bgr)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break

        time.sleep(0.002)

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
