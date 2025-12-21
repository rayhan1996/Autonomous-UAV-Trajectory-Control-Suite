import asyncio
from mavsdk import System

async def connect_px4(system_address: str) -> System:
    drone = System()
    await drone.connect(system_address=system_address)

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected!")
            break

    return drone

async def wait_armable(drone: System, sleep_s: float = 0.5) -> None:
    print("Waiting for drone to be armable...")
    async for health in drone.telemetry.health():
        if health.is_armable:
            print("Drone health OK. Ready to arm!")
            break
        await asyncio.sleep(sleep_s)
