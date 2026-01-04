# server/api/websocket.py
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from typing import List
import json
import asyncio

router = APIRouter()

class ConnectionManager:
    """WebSocket 연결 관리자"""

    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        """클라이언트 연결"""
        await websocket.accept()
        self.active_connections.append(websocket)
        print(f"Client connected. Total: {len(self.active_connections)}")

    def disconnect(self, websocket: WebSocket):
        """클라이언트 연결 해제"""
        self.active_connections.remove(websocket)
        print(f"Client disconnected. Total: {len(self.active_connections)}")

    async def send_personal(self, message: dict, websocket: WebSocket):
        """특정 클라이언트에게 전송"""
        await websocket.send_json(message)

    async def broadcast(self, message: dict):
        """모든 클라이언트에게 전송"""
        for connection in self.active_connections:
            await connection.send_json(message)

manager = ConnectionManager()

@router.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket 엔드포인트"""
    await manager.connect(websocket)

    try:
        while True:
            # 클라이언트로부터 메시지 수신
            data = await websocket.receive_json()

            print(f"Received: {data}")

            # 메시지 타입에 따라 처리
            msg_type = data.get("type")

            if msg_type == "ping":
                # Pong 응답
                await manager.send_personal({"type": "pong"}, websocket)

            elif msg_type == "joystick":
                # 조이스틱 입력 처리
                x = data.get("x", 0)
                y = data.get("y", 0)

                # 로봇 제어 (예시)
                # robot.move(x, y)

                # 응답
                await manager.send_personal({
                    "type": "joystick_ack",
                    "x": x,
                    "y": y
                }, websocket)

            elif msg_type == "command":
                # 명령 처리
                command = data.get("command")
                # 실행...

                await manager.send_personal({
                    "type": "command_result",
                    "success": True
                }, websocket)

    except WebSocketDisconnect:
        manager.disconnect(websocket)
    except Exception as e:
        print(f"WebSocket error: {e}")
        manager.disconnect(websocket)
