import gzip
import os
import re

def sanitize_name(name):
    # Replaces special characters with underscores and keeps names valid for C
    return re.sub(r'[^a-zA-Z0-9]', '_', name).upper()

def generate_assets():
    dist_path = "can-sniffer/dist"
    output_header = "../firmware/main/include/web_assets.h"
    
    if not os.path.exists(dist_path):
        print(f"Dist folder not found at: {dist_path}")
        return

    with open(output_header, 'w') as f:
        # Header start with Include Guards and necessary Standard C headers
        f.write("// Auto-generated web assets - DO NOT EDIT\n")
        f.write("#ifndef WEB_ASSETS_H\n")
        f.write("#define WEB_ASSETS_H\n\n")
        f.write("#include <stdint.h>\n")   # For uint8_t
        f.write("#include <stddef.h>\n")   # For size_t
        f.write("#include <stdbool.h>\n\n") # For bool
        
        for root, dirs, files in os.walk(dist_path):
            for file in files:
                full_path = os.path.join(root, file)
                rel_path = os.path.relpath(full_path, dist_path).replace("\\", "/")
                var_name = sanitize_name(rel_path)
                
                with open(full_path, 'rb') as f_in:
                    data = f_in.read()
                
                # Compress logic
                if not file.lower().endswith(('.mp4', '.png', '.jpg', '.jpeg', '.woff2')):
                    data = gzip.compress(data, compresslevel=9)
                    is_gz = True
                else:
                    is_gz = False

                # Convert binary to hex array
                hex_array = ", ".join([f"0x{b:02x}" for b in data])
                
                f.write(f"// Original File: {rel_path}\n")
                # Using 'const' ensures the data is stored in Flash (DROM) on ESP32
                f.write(f"const uint8_t {var_name}[] = {{{hex_array}}};\n")
                f.write(f"const size_t {var_name}_LEN = {len(data)};\n")
                f.write(f"const bool {var_name}_IS_GZ = {'true' if is_gz else 'false'};\n\n")

        f.write("#endif // WEB_ASSETS_H\n")

    print(f"Generated {output_header} successfully.")

if __name__ == "__main__":
    generate_assets()