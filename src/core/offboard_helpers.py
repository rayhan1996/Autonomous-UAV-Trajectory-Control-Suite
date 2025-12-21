import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError

async def prestream_position_setpoints(drone: System, down_m: float, yaw_deg: float = 0.0, n: int = 20):
    for _ in range(n):
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, down_m, yaw_deg))
        await asyncio.sleep(0.05)

async def start_offboard(drone: System) -> bool:
    try:
        await drone.offboard.start()
        print("Offboard started!")
        return True
    except OffboardError as e:
        print(f"Failed to start offboard: {e._result.result}")
        return False

async def stop_offboard_and_land(drone: System, sleep_s: float = 5.0):
    try:
        await drone.offboard.stop()
    except Exception:
        pass

    print("Landing...")
    await drone.action.land()
    await asyncio.sleep(sleep_s)
