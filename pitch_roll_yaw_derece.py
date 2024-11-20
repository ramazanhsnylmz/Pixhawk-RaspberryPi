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
            pitch = msg.pitch * (180 / 3.141592653589793)
            roll = msg.roll * (180 / 3.141592653589793)
            yaw = msg.yaw * (180 / 3.141592653589793)
            yaw = yaw % 360
            print(f"Pitch: {pitch:.2f} derece | Roll: {roll:.2f} derece | Yaw: {yaw:.2f} derece")
        else:
            print("ATTITUDE mesajı alınamadı.")
        
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Program sonlandırıldı.")
