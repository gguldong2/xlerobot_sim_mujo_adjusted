# server/api/robot.py
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Optional
import sys
sys.path.append('../../src')
from robots.so100 import SO100Arm

router = APIRouter(prefix="/api/robot", tags=["robot"])

# 전역 로봇 인스턴스 (나중에 의존성 주입으로 개선)
robot = None

class JointPosition(BaseModel):
    """조인트 각도"""
    joint_id: int
    angle: float  # degrees

class EEPosition(BaseModel):
    """엔드 이펙터 위치"""
    x: float
    y: float
    z: Optional[float] = None

class GripperCommand(BaseModel):
    """그리퍼 명령"""
    action: str  # "open" or "close"

# === 연결 관리 ===

@router.post("/connect")
async def connect(port: str = "/dev/ttyUSB0"):
    """로봇 연결"""
    global robot
    try:
        robot = SO100Arm(port=port)
        return {"status": "connected", "port": port}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/disconnect")
async def disconnect():
    """로봇 연결 해제"""
    global robot
    if robot:
        robot.cleanup()
        robot = None
        return {"status": "disconnected"}
    raise HTTPException(status_code=400, detail="No active connection")

@router.get("/status")
async def get_status():
    """로봇 상태 조회"""
    if not robot:
        raise HTTPException(status_code=400, detail="Robot not connected")

    return {
        "connected": True,
        "joint_count": 6,
        "gripper_state": "unknown"  # 실제로는 읽기
    }

# === 조인트 제어 ===

@router.post("/joint")
async def set_joint(pos: JointPosition):
    """단일 조인트 제어"""
    if not robot:
        raise HTTPException(status_code=400, detail="Robot not connected")

    try:
        robot.set_joint_angle(pos.joint_id, pos.angle)
        return {"joint_id": pos.joint_id, "angle": pos.angle}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/joint/{joint_id}")
async def get_joint(joint_id: int):
    """조인트 각도 읽기"""
    if not robot:
        raise HTTPException(status_code=400, detail="Robot not connected")

    angle = robot.get_joint_angle(joint_id)
    return {"joint_id": joint_id, "angle": angle}

# === 엔드 이펙터 제어 ===

@router.post("/ee")
async def set_ee_position(pos: EEPosition):
    """엔드 이펙터 위치 제어"""
    if not robot:
        raise HTTPException(status_code=400, detail="Robot not connected")

    try:
        if pos.z is not None:
            robot.set_ee_position([pos.x, pos.y, pos.z])
        else:
            robot.set_ee_position([pos.x, pos.y])

        return {"x": pos.x, "y": pos.y, "z": pos.z}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/ee")
async def get_ee_position():
    """엔드 이펙터 위치 읽기"""
    if not robot:
        raise HTTPException(status_code=400, detail="Robot not connected")

    pos = robot.get_ee_position()
    return {"x": pos[0], "y": pos[1], "z": pos[2] if len(pos) > 2 else None}

# === 그리퍼 제어 ===

@router.post("/gripper")
async def control_gripper(cmd: GripperCommand):
    """그리퍼 제어"""
    if not robot:
        raise HTTPException(status_code=400, detail="Robot not connected")

    if cmd.action == "open":
        robot.open_gripper()
    elif cmd.action == "close":
        robot.close_gripper()
    else:
        raise HTTPException(status_code=400, detail="Invalid action")

    return {"action": cmd.action}

# === 프리셋 동작 ===

@router.post("/home")
async def go_home():
    """홈 위치로 이동"""
    if not robot:
        raise HTTPException(status_code=400, detail="Robot not connected")

    robot.go_home()
    return {"status": "homing"}

@router.post("/sleep")
async def go_sleep():
    """슬립 자세"""
    if not robot:
        raise HTTPException(status_code=400, detail="Robot not connected")

    robot.go_sleep()
    return {"status": "sleeping"}
