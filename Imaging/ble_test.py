import asyncio
from bleak import BleakScanner

possible = ["19B10010-E8F2-537E-4F6C-D104768A1214: None", "19B10012-E8F2-537E-4F6C-D104768A1214: None", "C640E249-BA3A-504A-DD07-6C31BA2162B0: None"]

async def main():
    devices = await BleakScanner.discover()
    print(len(devices))
    for d in devices:
        if str(d) in possible:
            print()
            print("FOUND")
            print()
        print(d)

asyncio.run(main())