"""Read-only scan of Feetech STS servos on the bus. Never writes, never enables torque."""
import sys
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem5AE60826921"
BAUD = 1_000_000

# STS3215 control table addresses (read-only use)
REG = {
    "max_volt_limit": (14, 1),
    "min_volt_limit": (15, 1),
    "max_torque": (16, 2),
    "torque_enable": (40, 1),
    "position": (56, 2),
    "load": (60, 2),
    "voltage": (62, 1),
    "temp_C": (63, 1),
    "current": (69, 2),
}

port = PortHandler(PORT)
pkt = PacketHandler(0)
if not port.openPort() or not port.setBaudRate(BAUD):
    sys.exit(f"Could not open {PORT} @ {BAUD}")

found = []
for sid in range(1, 21):
    model, res, err = pkt.ping(port, sid)
    if res != COMM_SUCCESS:
        continue
    row = {"id": sid, "model": model}
    for name, (addr, size) in REG.items():
        read = pkt.read1ByteTxRx if size == 1 else pkt.read2ByteTxRx
        val, r, _ = read(port, sid, addr)
        row[name] = val if r == COMM_SUCCESS else None
    for k in ("voltage", "max_volt_limit", "min_volt_limit"):
        if row[k] is not None:
            row[k] = row[k] / 10  # 0.1 V units
    found.append(row)
    print(row)

port.closePort()
print(f"\n{len(found)} servo(s) found on {PORT}")
