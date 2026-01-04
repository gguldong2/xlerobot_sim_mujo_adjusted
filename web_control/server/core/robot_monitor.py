# server/core/robot_monitor.py
import asyncio
from api.websocket import manager

async def robot_status_loop(robot):
    """로봇 상태 주기적 전송"""
    while True:
        if robot and manager.active_connections:
            # 로봇 상태 읽기
            ee_pos = robot.get_ee_position()

            # 모든 클라이언트에게 전송
            await manager.broadcast({
                "type": "robot_status",
                "ee_position": {
                    "x": ee_pos[0],
                    "y": ee_pos[1],
                    "z": ee_pos[2] if len(ee_pos) > 2 else None
                },
                "timestamp": asyncio.get_event_loop().time()
            })

        await asyncio.sleep(0.1)  # 10Hz

# main.py에서 백그라운드 태스크로 실행
@app.on_event("startup")
async def startup_event():
    # 백그라운드 태스크 시작
    asyncio.create_task(robot_status_loop(robot))
