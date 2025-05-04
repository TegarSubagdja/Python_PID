import cv2
import time
import numpy as np
from PID import PID

# Inisialisasi PID arah saja
pid_arah = PID(Kp=1.5, Ki=0.01, Kd=0.5)
dt = 0.1
setpoint_arah = 0
frame = np.zeros((400, 600, 3), dtype=np.uint8)

def nothing(x):
    pass

cv2.namedWindow("Simulasi Arah")

# Trackbar untuk Error Arah (-50 hingga +50)
cv2.createTrackbar("Error Arah", "Simulasi Arah", 50, 100, nothing)  # 0 = -50, 100 = +50

# Trackbar untuk parameter PID Arah
cv2.createTrackbar("Kp", "Simulasi Arah", int(pid_arah.Kp * 10), 100, nothing)
cv2.createTrackbar("Ki", "Simulasi Arah", int(pid_arah.Ki * 100), 100, nothing)
cv2.createTrackbar("Kd", "Simulasi Arah", int(pid_arah.Kd * 10), 100, nothing)

while True:
    # Ambil nilai error arah dari trackbar
    raw_val = cv2.getTrackbarPos("Error Arah", "Simulasi Arah")
    error_arah = raw_val - 50  # Skala: -50 (terlalu kiri) sampai +50 (terlalu kanan)

    # Ambil nilai parameter PID yang diatur user
    pid_arah.Kp = cv2.getTrackbarPos("Kp", "Simulasi Arah") / 10.0
    pid_arah.Ki = cv2.getTrackbarPos("Ki", "Simulasi Arah") / 10.0
    pid_arah.Kd = cv2.getTrackbarPos("Kd", "Simulasi Arah") / 10.0

    # Hitung koreksi arah
    koreksi = pid_arah.compute(setpoint_arah, error_arah, dt)

    # Hitung simulasi kecepatan motor (semakin beda koreksi, motor akan beda)
    kecepatan_motorkiri = max(min(0 + koreksi, 255), 0)
    kecepatan_motorkanan = max(min(0 - koreksi, 255), 0)

    # Update UI
    frame[:] = (30, 30, 30)
    cv2.putText(frame, f"Error Arah: {error_arah}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.putText(frame, f"Kp: {pid_arah.Kp:.2f} Ki: {pid_arah.Ki:.2f} Kd: {pid_arah.Kd:.2f}", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 255, 100), 2)
    cv2.putText(frame, f"Motor Kiri : {kecepatan_motorkiri:.2f}", (20, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
    cv2.putText(frame, f"Motor Kanan: {kecepatan_motorkanan:.2f}", (20, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)

    # Tampilan visual garis tengah dan koreksi arah
    cv2.line(frame, (300, 300), (300 + int(koreksi * 2), 300), (0, 0, 255), 4)

    cv2.imshow("Simulasi Arah", frame)

    # Keluar jika ESC ditekan
    if cv2.waitKey(1) == 27:
        print("Keluar dari simulasi.")
        break

    time.sleep(dt)

cv2.destroyAllWindows()
