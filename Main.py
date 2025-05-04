import cv2
import time
import numpy as np
from PID import PID

# Inisialisasi PID dengan nilai default
pid_arah = PID(Kp=1.5, Ki=0.01, Kd=0.5)
pid_jarak = PID(Kp=2.0, Ki=0.02, Kd=0.8)

dt = 0.1
setpoint_arah = 0
setpoint_jarak = 0
measurement_jarak = 30
frame = np.zeros((400, 600, 3), dtype=np.uint8)

def nothing(x):
    pass

# Buat jendela dan trackbar
cv2.namedWindow("Simulasi PID")
cv2.createTrackbar("Error Arah", "Simulasi PID", 50, 100, nothing)  # -50 to +50

# Trackbar untuk PID Arah
cv2.createTrackbar("Kp", "Simulasi PID", int(pid_arah.Kp * 10), 100, nothing)   # 0.0 - 10.0
cv2.createTrackbar("Ki", "Simulasi PID", int(pid_arah.Ki * 100), 100, nothing)  # 0.00 - 1.00
cv2.createTrackbar("Kd", "Simulasi PID", int(pid_arah.Kd * 10), 100, nothing)   # 0.0 - 10.0

while True:
    # Ambil nilai dari trackbar dan konversi
    slider_val = cv2.getTrackbarPos("Error Arah", "Simulasi PID")
    measurement_arah = slider_val - 50

    # Ambil parameter PID baru dari trackbar dan update PID controller
    pid_arah.Kp = cv2.getTrackbarPos("Kp", "Simulasi PID") / 10.0
    pid_arah.Ki = cv2.getTrackbarPos("Ki", "Simulasi PID") / 10.0
    pid_arah.Kd = cv2.getTrackbarPos("Kd", "Simulasi PID") / 10.0

    # Hitung PID output
    koreksi_arah = pid_arah.compute(setpoint_arah, measurement_arah, dt)
    kecepatan_maju = pid_jarak.compute(setpoint_jarak, measurement_jarak, dt)

    motor_kiri = max(min(kecepatan_maju + koreksi_arah, 255), 0)
    motor_kanan = max(min(kecepatan_maju - koreksi_arah, 255), 0)

    # Visualisasi ke UI
    frame[:] = (0, 0, 0)
    cv2.putText(frame, f"Error Arah: {measurement_arah:.2f}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.putText(frame, f"Kp: {pid_arah.Kp:.2f} Ki: {pid_arah.Ki:.2f} Kd: {pid_arah.Kd:.2f}", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 255, 100), 2)
    cv2.putText(frame, f"Motor Kiri: {motor_kiri:.2f}", (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(frame, f"Motor Kanan: {motor_kanan:.2f}", (20, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(frame, f"Jarak ke Target: {measurement_jarak:.2f} cm", (20, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

    cv2.imshow("Simulasi PID", frame)

    if cv2.waitKey(1) == 27:
        print("ESC ditekan, keluar dari simulasi.")
        break

    measurement_jarak = max(measurement_jarak - 1.5, 0)
    time.sleep(dt)

cv2.destroyAllWindows()
