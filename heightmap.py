import struct
from PIL import Image

def export_final_map(bin_file, width=128, height=128):
    with open(bin_file, "rb") as f:
        data = f.read()

    pixels = []
    # Read the whole 32KB file as 16-bit Big Endian
    for i in range(0, len(data) - 1, 2):
        val = struct.unpack(">H", data[i:i+2])[0]
        # Normalize to 8-bit for a standard PNG
        pixels.append(val >> 8)

    # Create image
    img = Image.new('L', (width, height))
    img.putdata(pixels[:width*height])
    
    # Scale up for a high-res view
    img = img.resize((1024, 1024), Image.LANCZOS)
    img.save("FINAL_CLEAN_HEIGHTMAP.png")
    print("Export Complete: FINAL_CLEAN_HEIGHTMAP.png")

if __name__ == "__main__":
    export_final_map(r"C:\Users\istuart\Downloads\decompressed_assets\asset_00949B3C.bin")