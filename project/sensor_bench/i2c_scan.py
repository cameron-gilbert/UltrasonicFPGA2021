"""Verify the MCP2221A bridge is visible and scan the I2C bus for the BNO085.

Wire the BNO085 to the MCP2221A over STEMMA QT, plug the MCP2221A into the
laptop with a USB-C DATA cable, then run (PowerShell, from this folder):

    $env:BLINKA_MCP2221 = "1"
    python i2c_scan.py

Expected: the BNO085 answers at 0x4A (default) or 0x4B (ADR pin high).
"""
import board
import busio

i2c = busio.I2C(board.SCL, board.SDA)

while not i2c.try_lock():
    pass

try:
    addrs = i2c.scan()
    print("I2C devices found:", [hex(a) for a in addrs])
    if 0x4a in addrs or 0x4b in addrs:
        print("BNO085 detected.")
    else:
        print("BNO085 NOT found - check STEMMA QT seating and the USB DATA cable.")
finally:
    i2c.unlock()
