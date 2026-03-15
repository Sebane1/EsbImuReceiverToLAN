import sys

files = [
    (0x0000, r".pio\build\esp32-s3-devkitc-1\bootloader.bin"),
    (0x8000, r".pio\build\esp32-s3-devkitc-1\partitions.bin"),
    (0xe000, r"I:\PlatformIO\packages\framework-arduinoespressif32\tools\partitions\boot_app0.bin"),
    (0x10000, r".pio\build\esp32-s3-devkitc-1\firmware.bin")
]

out_data = bytearray()

for offset, path in files:
    with open(path, 'rb') as f:
        data = f.read()
    if len(out_data) < offset:
        out_data.extend(b'\xff' * (offset - len(out_data)))
    
    # Pad out_data if we are overwriting past the end
    if len(out_data) < offset + len(data):
        out_data.extend(b'\xff' * (offset + len(data) - len(out_data)))
        
    out_data[offset:offset+len(data)] = data

with open('merged.bin', 'wb') as f:
    f.write(out_data)

print("Merged successfully!")
