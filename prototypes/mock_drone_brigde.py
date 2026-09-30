import asyncio
import json
import random
import websockets

RELAY_URL = "wss://uav-rescue-relay.onrender.com/ws/drone"


async def send_fake_telemetry(ws):
    while True:
        telemetry = {
            "type": "telemetry",
            "battery_pct": round(random.uniform(40, 100), 1),
            "lat": 10.762622 + random.uniform(-0.001, 0.001),
            "lon": 106.660172 + random.uniform(-0.001, 0.001),
            "armed": True,
            "mode": "AUTO.MISSION",
        }
        await ws.send(json.dumps(telemetry))
        await asyncio.sleep(2)


async def listen_for_commands(ws):
    async for message in ws:
        command = json.loads(message)
        print(f"Received command from app: {command}")
        # Later: call a ROS2 service / publish a topic here


async def main():
    async with websockets.connect(RELAY_URL) as ws:
        print("Connected to relay as drone bridge.")
        await asyncio.gather(send_fake_telemetry(ws), listen_for_commands(ws))


if __name__ == "__main__":
    asyncio.run(main())