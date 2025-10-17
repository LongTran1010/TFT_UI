import serial
import csv
from datetime import datetime

# === Cấu hình cổng COM ===
# Kiểm tra COM nào ESP32 đang kết nối (xem góc phải dưới PlatformIO: COM3, COM4, ...)
PORT = "COM3"      # đổi theo cổng của bạn
BAUD = 115200      # tốc độ Serial

# === Tên file CSV (tự tạo theo thời gian) ===
filename = f"tof_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

# === Mở cổng serial và file ghi ===
ser = serial.Serial(PORT, BAUD, timeout=1)
with open(filename, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["time_ms","distance_raw_m","distance_filt_m","strength","fps","start parse frame","render_time_ms"])

    print(f"[OK] Đang ghi dữ liệu vào {filename}")
    try:
        while True:
            line = ser.readline().decode(errors='ignore').strip()
            if not line:
                continue
            parts = line.split(",")
            if len(parts) == 5:
                writer.writerow([parts[0], parts[1], parts[2], parts[3], parts[4], "", ""])
            elif len(parts)==2 and parts[0] in ("Start parse frame","render TFT"):  # marker bắt đầu và kết thúc render
                writer.writerow(["","","","","", parts[0], parts[1]])
    except KeyboardInterrupt:
        print("\n[STOP] Dừng ghi dữ liệu.")
