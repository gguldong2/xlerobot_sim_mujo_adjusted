# server/main.py
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from api import robot

from api import websocket
app.include_router(websocket.router)

from api import camera
app.include_router(camera.router)

app = FastAPI(
    title="XLeRobot Web Control API",
    version="1.0.0",
    description="로봇 원격 제어 REST API"
)

# CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 라우터 등록
app.include_router(robot.router)

@app.get("/")
async def root():
    return {
        "name": "XLeRobot Web Control",
        "version": "1.0.0",
        "docs": "/docs"
    }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
