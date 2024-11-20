import sys
sys.path.insert(0, "/home/pi/mis/lib/python3.11/site-packages")
from pymavlink import mavutil
import threading
import math
import time


######## MOTOR ########
# Pixhawk ile bağlantı kurulumu
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
master.wait_heartbeat()
print("Pixhawk ile bağlantı sağlandı!")

# Motor PWM sınırları
MIN_PWM = 1000
MAX_PWM = 2000
STOP_PWM = 1500  # Durma pozisyonu


#ARM
def arm():

    print("Waiting for vehicle to become armable...")
    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        if msg.system_status == mavutil.mavlink.MAV_STATE_STANDBY:
            break
        time.sleep(1)
    print("Vehicle is now armable")
    print("Arming motors...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # Confirmation
        1,  # Arm
        0, 0, 0, 0, 0, 0
    )
    # Arming tamamlanana kadar bekle
    timeout = time.time() + 10  # 10 saniye bekle
    while time.time() < timeout:
        msg = master.recv_match(type='HEARTBEAT', blocking=False)
        if msg and msg.system_status == mavutil.mavlink.MAV_STATE_ACTIVE:
            print("Vehicle is now armed!")
            return
        time.sleep(1)
    print("Failed to arm the vehicle within the timeout period.")



#DISARM
def disarm():
    print("Disarming motors...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # Confirmation
        0,  # Disarm
        0, 0, 0, 0, 0, 0
    )
    # Disarming tamamlanana kadar bekle
    timeout = time.time() + 10  # 10 saniye bekle
    while time.time() < timeout:
        msg = master.recv_match(type='HEARTBEAT', blocking=False)
        if msg and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_DECODE_ARMED):
            print("Vehicle is now disarmed!")
            return
        time.sleep(1)
    print("Failed to disarm the vehicle within the timeout period.")


#PWM SINYALI ISTENILEN DEGERE ULASTIRMA
def gradual_pwm_change(start_pwm, end_pwm, duration):
    num_steps = MAX_PWM - MIN_PWM
    half_duration = duration / 2
    step_duration = half_duration / num_steps  # Artış ve azalış süresi
    # PWM değerlerini artır
    start_time = time.time()
    while time.time() - start_time < half_duration:
        elapsed_time = time.time() - start_time
        step_progress = elapsed_time / half_duration
        current_pwm = [int(start + (end - start) * step_progress) for start, end in zip(start_pwm, end_pwm)]
        # PWM değerlerini sınırlar içinde tutma
        current_pwm = [max(MIN_PWM, min(MAX_PWM, pwm)) for pwm in current_pwm]
        set_motor_pwm(current_pwm)
        time.sleep(step_duration)
    # PWM değerlerini azalt
    start_time = time.time()
    while time.time() - start_time < half_duration:
        elapsed_time = time.time() - start_time
        step_progress = elapsed_time / half_duration
        current_pwm = [int(end - (end - start) * step_progress) for start, end in zip(start_pwm, end_pwm)]
        # PWM değerlerini sınırlar içinde tutma
        current_pwm = [max(MIN_PWM, min(MAX_PWM, pwm)) for pwm in current_pwm]
        set_motor_pwm(current_pwm)
        time.sleep(step_duration)
    # PWM değerlerini sıfırlama (opsiyonel)
    set_motor_pwm([STOP_PWM] * 8)  # Tüm motorları durdur



#ISTENILEN PWM DEGERI VERME
def set_motor_pwm(pwm_values):
    if len(pwm_values) != 8:
        raise ValueError("PWM değerleri listesi 8 uzunluğunda olmalıdır.")
    if any(pwm < MIN_PWM or pwm > MAX_PWM for pwm in pwm_values):
        raise ValueError(f"PWM değeri {MIN_PWM} ile {MAX_PWM} arasında olmalıdır.")
    print(f"Setting motor PWM values: {pwm_values}")
    for i, pwm in enumerate(pwm_values):
        master.mav.command_long_send(
            master.target_system,  # target_system
            master.target_component,  # target_component
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
            0,  #confirmation
            i + 1,  #servo_number (1 indexed)
            pwm,  #pwm_value
            0, 0, 0, 0, 0, 0  # unused parameters
        )


#ISTENILEN PWM SINYALINI ISTENILEN SUREDE VERME
def set_motor_pwm_with_duration(pwm_values, duration):
    if len(pwm_values) != 8:
        raise ValueError("PWM değerleri listesi 8 uzunluğunda olmalıdır.")
    if any(pwm < MIN_PWM or pwm > MAX_PWM for pwm in pwm_values):
        raise ValueError(f"PWM değeri {MIN_PWM} ile {MAX_PWM} arasında olmalıdır.")
    print(f"Setting motor PWM values: {pwm_values} for {duration} seconds")
    start_time = time.time()
    while time.time() - start_time < duration:
        for i, pwm in enumerate(pwm_values):
            master.mav.command_long_send(
                master.target_system,  # target_system
                master.target_component,  # target_component
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
                0,  # confirmation
                i + 1,  # servo_number (1 indexed)
                pwm,  # pwm_value
                0, 0, 0, 0, 0, 0  # unused parameters
            )
        time.sleep(0.1)  # Motorları her 0.1 saniyede bir kontrol eder




#HAREKET FONKSIYONLARI
#ILERI
def move_forward(duration):
    start_time = time.time()
    while time.time() - start_time < duration:
        set_motor_pwm([1180, 1860, 1500, 1500, 1500, 1500, 1500, 1500])
        time.sleep(0.1)

#BATMA
def submerge(duration):
    start_time = time.time()
    while time.time() - start_time < duration:
        set_motor_pwm([1500, 1500, 1850, 1800, 1500, 1500, 1500, 1500])
        time.sleep(0.1)


#SOL DONME
def turn_left(duration):
    start_time = time.time()
    while time.time() - start_time < duration:
        set_motor_pwm([1320, 1500, 1500, 1500, 1500, 1500, 1500, 1500])
        time.sleep(0.1)


#SAG DONME
def turn_right(duration):
    start_time = time.time()
    while time.time() - start_time < duration:
        set_motor_pwm([1500, 1680, 1500, 1500, 1500, 1500, 1500, 1500])
        time.sleep(0.1)


#SOLLU GITME
def go_left(duration):
    start_time = time.time()
    while time.time() - start_time < duration:
        set_motor_pwm([1300, 1500, 1500, 1500, 1500, 1580, 1500, 1500])
        time.sleep(0.1)


#SAGLI GITME
def go_right(duration):
    start_time = time.time()
    while time.time() - start_time < duration:
        set_motor_pwm([1500, 1700, 1500, 1500, 1580, 1500, 1500, 1500])
        time.sleep(0.1)


#YUKARI CIKMA
def go_up(duration):
    start_time = time.time()
    while time.time() - start_time < duration:
        set_motor_pwm([1500, 1700, 1500, 1500, 1500, 1500, 1300, 1500])
        time.sleep(0.1)



#TORPIDO
def fire_torpedo(duration):
    start_time = time.time()
    while time.time() - start_time < duration:
        set_motor_pwm([1500, 1500, 1500, 1500, 1500, 1500, 2000, 1500])
        time.sleep(0.1)
    set_motor_pwm([STOP_PWM] * 8)  # Torpido fırlatma tamamlandığında motorları durdur


#STOP
def stop_motors():
    print("Stopping all mo*tors...")
    set_motor_pwm([STOP_PWM] * 8)  # Durma PWM değeri olarak STOP_PWM kullanılacak


#ANA BASLANGIC
time.sleep(2)
submerge(8)
stop_motors()
#ANA BITIS

#NOTLAR


