#!/usr/bin/env python3
"""最小链接测试：只连、只打印，不动飞机"""
import asyncio
from mavsdk import System

ADDRESS = "serial:///dev/ttyACM0:57600"   # ← 按你的连接方式改

async def main():
    drone = System()
    await drone.connect(system_address=ADDRESS)

    print("⏳ 等待连接...")
    async for st in drone.core.connection_state():
        if st.is_connected:
            print("✅ 已连接飞控")
            break

    v = await drone.info.get_version()
    print(f"📟 飞控固件版本: {v.flight_sw_major}.{v.flight_sw_minor}.{v.flight_sw_patch}")

    print("🧭 健康状态（Ctrl+C 退出）")
    async for h in drone.telemetry.health():
        print(f"  local_pos_ok={h.is_local_position_ok}  "
              f"global_pos_ok={h.is_global_position_ok}  "
              f"home_pos_ok={h.is_home_position_ok}")

if __name__ == "__main__":
    asyncio.run(main())