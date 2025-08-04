import asyncio
import platform
import threading
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from bleak import BleakScanner
import statistics
import math
import socket
import time
import numpy as np

# --- CONFIG ---
TARGET_MAC = "00:00:00:00:05:30".upper()
TX_POWER = -59
PATH_LOSS_EXPONENT = 2.0
CALIBRATION_OFFSET = 1.0

RSSI_WINDOW = 5
HISTORY_LEN = 100

# Anchor positions in 2D: PC, Mobile1, Mobile2
pos_pc = (0.0, 0.0)
pos_mobile1 = (3.0, 0.0)
pos_mobile2 = (1.5, 2.6)  # adjust to your actual geometry

# Histories
rssi_history_pc = deque(maxlen=RSSI_WINDOW)
rssi_history_mobile1 = deque(maxlen=RSSI_WINDOW)
rssi_history_mobile2 = deque(maxlen=RSSI_WINDOW)
distances_pc_kalman = deque(maxlen=HISTORY_LEN)
distances_mobile1_kalman = deque(maxlen=HISTORY_LEN)
distances_mobile2_kalman = deque(maxlen=HISTORY_LEN)
position_history = deque(maxlen=HISTORY_LEN)

last_reported_pc = None
last_reported_mobile1 = None
last_reported_mobile2 = None
last_reported_pos = None

class Kalman1D:
    def __init__(self, q, r, initial=None):
        self.Q = q
        self.R = r
        self.x = initial
        self.P = 1.0

    def update(self, meas):
        if self.x is None:
            self.x = meas
            return self.x
        self.P = self.P + self.Q
        K = self.P / (self.P + self.R)
        self.x = self.x + K * (meas - self.x)
        self.P = (1 - K) * self.P
        return self.x

kalman_pc = Kalman1D(0.01, 0.5)
kalman_mobile1 = Kalman1D(0.01, 0.5)
kalman_mobile2 = Kalman1D(0.01, 0.5)

def calculate_distance(rssi):
    return 10 ** ((TX_POWER - rssi) / (10 * PATH_LOSS_EXPONENT)) + CALIBRATION_OFFSET

def filter_outliers(lst):
    if not lst:
        return []
    med = statistics.median(lst)
    deviations = [abs(x - med) for x in lst]
    mad = statistics.median(deviations) if deviations else 0
    threshold = 2 * mad if mad > 0 else 5
    filtered = [x for x in lst if abs(x - med) <= threshold]
    return filtered if filtered else lst

def trilateration_least_squares(anchors, distances):
    x1, y1 = anchors[0]
    d1 = distances[0]
    A = []
    b = []
    for (xi, yi), di in zip(anchors[1:], distances[1:]):
        A.append([2*(xi - x1), 2*(yi - y1)])
        b.append((d1**2 - di**2) - (x1**2 - xi**2) - (y1**2 - yi**2))
    A = np.array(A)
    b = np.array(b)
    try:
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
        return (sol[0], sol[1])
    except Exception:
        return None

# UDP listener expects messages like "M1,MAC,RSSI" and "M2,MAC,RSSI"
UDP_IP = "0.0.0.0"
UDP_PORT = 5005

def udp_listener():
    global last_reported_mobile1, last_reported_mobile2
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(1.0)
    print(f"[{time.strftime('%H:%M:%S')}] UDP listener on {UDP_IP}:{UDP_PORT}")
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            msg = data.decode(errors="replace").strip()
            parts = [p.strip() for p in msg.split(",", 2)]
            if len(parts) != 3:
                continue
            tag, mac, rssi_s = parts
            if mac.upper() != TARGET_MAC:
                continue
            try:
                rssi = float(rssi_s)
            except ValueError:
                continue
            if tag == "M1":
                rssi_history_mobile1.append(rssi)
                if len(rssi_history_mobile1) < RSSI_WINDOW:
                    continue
                window = filter_outliers(list(rssi_history_mobile1))
                avg = sum(window) / len(window)
                dist_raw = calculate_distance(avg)
                dist_k = kalman_mobile1.update(dist_raw)
                distances_mobile1_kalman.append(dist_k)
                if (last_reported_mobile1 is None or
                    abs(dist_k - last_reported_mobile1)/max(last_reported_mobile1,1e-6) > 0.05):
                    print(f"[{time.strftime('%H:%M:%S')}] [Mobile1] RSSI avg: {avg:.1f} -> dist Kalman: {dist_k:.2f} m")
                    last_reported_mobile1 = dist_k
            elif tag == "M2":
                rssi_history_mobile2.append(rssi)
                if len(rssi_history_mobile2) < RSSI_WINDOW:
                    continue
                window = filter_outliers(list(rssi_history_mobile2))
                avg = sum(window) / len(window)
                dist_raw = calculate_distance(avg)
                dist_k = kalman_mobile2.update(dist_raw)
                distances_mobile2_kalman.append(dist_k)
                if (last_reported_mobile2 is None or
                    abs(dist_k - last_reported_mobile2)/max(last_reported_mobile2,1e-6) > 0.05):
                    print(f"[{time.strftime('%H:%M:%S')}] [Mobile2] RSSI avg: {avg:.1f} -> dist Kalman: {dist_k:.2f} m")
                    last_reported_mobile2 = dist_k
        except socket.timeout:
            continue
        except Exception as e:
            print("UDP error:", e)
            break
    sock.close()

def detection_callback(device, advertisement_data):
    global last_reported_pc
    if device.address.upper() != TARGET_MAC:
        return
    rssi = advertisement_data.rssi
    rssi_history_pc.append(rssi)
    if len(rssi_history_pc) < RSSI_WINDOW:
        return
    window = filter_outliers(list(rssi_history_pc))
    avg = sum(window) / len(window)
    dist_raw = calculate_distance(avg)
    dist_k = kalman_pc.update(dist_raw)
    distances_pc_kalman.append(dist_k)
    if (last_reported_pc is None or
        abs(dist_k - last_reported_pc)/max(last_reported_pc,1e-6) > 0.05):
        print(f"[{time.strftime('%H:%M:%S')}] [PC] RSSI avg: {avg:.1f} -> dist Kalman: {dist_k:.2f} m")
        last_reported_pc = dist_k

async def scan_ble():
    scanner = BleakScanner(detection_callback=detection_callback)
    await scanner.start()
    try:
        while True:
            await asyncio.sleep(0.5)
    finally:
        await scanner.stop()

def start_ble():
    if platform.system() == "Windows":
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
    asyncio.run(scan_ble())

def update_plot(frame):
    plt.cla()
    has_pc = len(distances_pc_kalman) > 0
    has_m1 = len(distances_mobile1_kalman) > 0
    has_m2 = len(distances_mobile2_kalman) > 0
    if not (has_pc and has_m1 and has_m2):
        plt.title("Waiting for all three anchors...")
        plt.grid(True)
        return

    d_pc = distances_pc_kalman[-1]
    d_m1 = distances_mobile1_kalman[-1]
    d_m2 = distances_mobile2_kalman[-1]

    anchors = [pos_pc, pos_mobile1, pos_mobile2]
    distances = [d_pc, d_m1, d_m2]
    pos_est = trilateration_least_squares(anchors, distances)
    if pos_est is None:
        plt.title("Trilateration failed")
        return

    x, y = pos_est
    position_history.append((x, y))

    global last_reported_pos
    if (last_reported_pos is None or
        math.hypot(x - last_reported_pos[0], y - last_reported_pos[1]) / (max(math.hypot(*last_reported_pos),1e-6)) > 0.05):
        print(f"[{time.strftime('%H:%M:%S')}] [Position] Estimated: X={x:.2f}, Y={y:.2f}")
        last_reported_pos = (x, y)

    xs = [p[0] for p in position_history]
    ys = [p[1] for p in position_history]
    plt.plot(xs, ys, label="Estimated position trace")
    plt.scatter([pos_pc[0], pos_mobile1[0], pos_mobile2[0]],
                [pos_pc[1], pos_mobile1[1], pos_mobile2[1]],
                marker='x', label="Anchors")
    plt.scatter(x, y, marker='o', label="Estimate", s=100)
    plt.title("3-anchor Trilateration Estimated Position")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.legend()
    plt.grid(True)

def main():
    threading.Thread(target=udp_listener, daemon=True).start()
    threading.Thread(target=start_ble, daemon=True).start()
    fig = plt.figure(figsize=(7,6))
    ani = animation.FuncAnimation(fig, update_plot, interval=500, cache_frame_data=False)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
