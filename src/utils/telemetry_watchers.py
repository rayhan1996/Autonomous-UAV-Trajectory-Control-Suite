from mavsdk import System
from .shared_state import SharedState

async def watch_posvel(drone: System, state: SharedState):
    async for data in drone.telemetry.position_velocity_ned():
        pos = data.position
        vel = data.velocity
        state.pos_ned = (pos.north_m, pos.east_m, pos.down_m)
        state.vel_ned = (vel.north_m_s, vel.east_m_s, vel.down_m_s)

        if not state.running:
            break

async def watch_attitude(drone: System, state: SharedState):
    async for att in drone.telemetry.attitude_euler():
        state.attitude_deg = (att.roll_deg, att.pitch_deg, att.yaw_deg)
        if not state.running:
            break

async def watch_flight_mode(drone: System, state: SharedState):
    async for mode in drone.telemetry.flight_mode():
        state.flight_mode = mode
        if not state.running:
            break
