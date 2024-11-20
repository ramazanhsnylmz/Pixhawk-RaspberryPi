import time
from pymavlink import mavutil

# Pixhawk ile bağlantı kurulumu
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
master.wait_heartbeat()
print("Pixhawk ile bağlantı sağlandı!")

# Sürekli olarak pitch, roll ve yaw değerlerini okuma
try:
    while True:
        print("Mesaj bekleniyor...")
        msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=1)
        
        if msg:
            print("ATTITUDE mesajı alındı!")
            pitch = msg.pitch
            roll = msg.roll
            yaw = msg.yaw
            print(f"Pitch: {pitch:.4f} rad | Roll: {roll:.4f} rad | Yaw: {yaw:.4f} rad")
        else:
            print("ATTITUDE mesajı alınamadı.")
        
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Program sonlandırıldı.")
