# server/api/camera.py
from fastapi import APIRouter
from fastapi.responses import StreamingResponse
import cv2
import asyncio

router = APIRouter(prefix="/api/camera", tags=["camera"])

class VideoCamera:
    """비디오 카메라"""

    def __init__(self, camera_id=0):
        self.cap = cv2.VideoCapture(camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    def get_frame(self):
        """프레임 읽기"""
        ret, frame = self.cap.read()
        if not ret:
            return None

        # JPEG 인코딩
        ret, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ret:
            return None

        return jpeg.tobytes()

    def cleanup(self):
        """정리"""
        self.cap.release()

# 전역 카메라
camera = VideoCamera()

def generate_frames():
    """프레임 생성기 (MJPEG)"""
    while True:
        frame = camera.get_frame()

        if frame is None:
            continue

        # MJPEG 형식
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@router.get("/stream")
async def video_stream():
    """비디오 스트림"""
    return StreamingResponse(
        generate_frames(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

@router.get("/snapshot")
async def snapshot():
    """스냅샷"""
    frame = camera.get_frame()

    if frame is None:
        return {"error": "Failed to capture"}

    return StreamingResponse(
        iter([frame]),
        media_type="image/jpeg"
    )
