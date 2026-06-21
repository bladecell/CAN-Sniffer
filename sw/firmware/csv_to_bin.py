import csv

RECORD_SIZE = 128
CODE_SIZE = 6
DESC_SIZE = RECORD_SIZE - CODE_SIZE

with open('obd-trouble-codes.csv', mode='r', encoding='utf-8') as f:

    rows = [r for r in csv.reader(f) if len(r) >= 2]

rows.sort(key=lambda x: x[0].strip())

with open('dtcs.bin', mode='wb') as out:
    for code, desc in rows:
        c_bytes = code.strip().encode('ascii', errors='ignore').ljust(CODE_SIZE, b'\0')
        d_bytes = desc.strip().encode('ascii', errors='ignore')[: (DESC_SIZE - 1)].ljust(DESC_SIZE, b'\0')
        out.write(c_bytes + d_bytes)
        
    final_size_kb = out.tell() / 1024  # <-- Grab the size while the door is still open

print(f"Success! Packed {len(rows)} DTCs into a {final_size_kb:.1f} KB binary file.")