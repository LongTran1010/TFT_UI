#!/usr/bin/env python3
import json
import csv
import time
import datetime as dt
import os
import paho.mqtt.client as mqtt

# ----- CONFIG -----
#MQTT_HOST = "192.168.90.50"     # SỬA: IP broker, trùng với ESP32
MQTT_HOST = "172.20.10.3/-"
MQTT_PORT = 1883
MQTT_TOPIC = "thesis/tc22/log"

OUT_DIR = "logs_tc22"          # thư mục lưu CSV

os.makedirs(OUT_DIR, exist_ok=True)

# Tạo tên file mới cho mỗi lần chạy
ts_str = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
csv_path = os.path.join(OUT_DIR, f"tc22_log_{ts_str}.csv")

csv_file = open(csv_path, "w", newline="", encoding="utf-8")
writer = csv.writer(csv_file)
# Header khớp với JSON publish từ ESP32
writer.writerow(["distance_m", "distance_filt_m", "t_rel_s", "dt_s", "fps", "host_ts"])

print(f"[LOGGER] Writing to {csv_path}")

def on_connect(client, userdata, flags, rc):
  if rc == 0:
    print("[LOGGER] Connected to MQTT, rc=0")
    client.subscribe(MQTT_TOPIC)
    print(f"[LOGGER] Subscribed to {MQTT_TOPIC}")
  else:
    print(f"[LOGGER] Failed to connect, rc={rc}")

def on_message(client, userdata, msg):
  try:
    payload = msg.payload.decode("utf-8")
    data = json.loads(payload)

    distance_m      = data.get("distance_m")
    distance_filt_m = data.get("distance_filt_m")
    t_rel_s         = data.get("t_rel_s")
    dt_s            = data.get("dt_s")
    fps             = data.get("fps")
    host_time         = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    writer.writerow([distance_m, distance_filt_m, t_rel_s, dt_s, fps, host_time])
    csv_file.flush()
  except Exception as e:
    print("[LOGGER] Parse error:", e)

def main():
  client = mqtt.Client(client_id="tc22-logger-pc")
  client.on_connect = on_connect
  client.on_message = on_message

  print(f"[LOGGER] Connecting to MQTT {MQTT_HOST}:{MQTT_PORT} ...")
  client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)

  try:
    print("[LOGGER] Logging... Press Ctrl+C to stop.")
    client.loop_forever()
  except KeyboardInterrupt:
    print("\n[LOGGER] Stopping, closing file...")
  finally:
    csv_file.close()
    client.disconnect()
    print("[LOGGER] Saved:", csv_path)

if __name__ == "__main__":
  main()
