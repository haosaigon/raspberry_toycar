from flask import Flask, render_template, request, jsonify
import pigpio
from kalman_filter import KalmanFilter
from servo_control_loop import servo_control
from map_degree_pwm import map_value
import math
import threading
import time
import logging
app = Flask(__name__)

# Initialize pigpio

pi = pigpio.pi()

#define timeout trigger for status code 408
timeout_happened = False
# GPIO pins for servos
STEERING_SERVO = 19
SPEED_SERVO = 18
GEAR_SERVO = 13
# Set pin modes
pi.set_mode(STEERING_SERVO, pigpio.OUTPUT)
pi.set_mode(SPEED_SERVO, pigpio.OUTPUT)
pi.set_mode(GEAR_SERVO, pigpio.OUTPUT)
# Fix the pwm of Traxxas motor
def map_value_speed(x):
    if x > 0:
        x = 1568 + x
    elif x < 0:
        x =  1470 + x
    else :
        x = 1500
    print("speed pulse:", x)
    return x

# Kalman filters for speed and steering
speed_filter = KalmanFilter(process_variance=0.01, measurement_variance=0.1)
turn_filter = KalmanFilter(process_variance=0.01, measurement_variance=0.1)

# Car state
car_state = {"speed": 0, "turn": 0, "gear": 0}

#reset if loss communication


@app.route('/')
def index():
    return render_template('1control.html')#check heartbeat
#@app.route("/heartbeat_client")
#def heartbeat_client():
    #return "OK", 200

@app.route('/control', methods=['POST'])
def control():
    data = request.json
    raw_speed = float(data.get('speed', 0))
    raw_turn = float(data.get('turn', 0))
    raw_gear = float(data.get('gear', 0))
    # Apply Kalman filters
    filtered_turn = turn_filter.update(raw_turn)
    # Update car state
    car_state["speed"] = raw_speed

    # loop turn value

    car_state["turn"] = filtered_turn

    # Convert filtered values to servo pulsewidth
    turn_pulse = map_value(filtered_turn, -45, 45, 1000, 2000)
    print("Turn pulse:", turn_pulse)
    gear_pulse = map_value(raw_gear, -100, 100, 1000, 2000)
    speed_pulse = map_value_speed(raw_speed)
    # Send PWM signals to servos
    pi.set_servo_pulsewidth(SPEED_SERVO, speed_pulse)
#    pi.set_servo_pulsewidth(STEERING_SERVO, turn_pulse)
    pi.set_servo_pulsewidth(GEAR_SERVO, gear_pulse)
# Set steering servo
    servo_control(STEERING_SERVO, raw_turn)
    print(f"[Filtered] Speed: {raw_speed:.2f} | Turn: {raw_turn:.2f} | Gear: {raw_gear:.2f}")
    with open("car_log.txt", "a") as f:
        log_line = f"[Filtered] Speed: {raw_speed:.2f} | Turn: {raw_turn:.2f} | Gear: {raw_gear:.2f}\n"
        f.write(log_line)
        print(log_line, end="")
    logging.basicConfig(
        filename="car_log.txt",
        level=logging.INFO,
        format="%(asctime)s - %(message)s"
    )

    logging.info(f"Speed: {raw_speed:.2f} | Turn: {raw_turn:.2f} | Gear: {raw_gear:.2f}")
    return jsonify({
        "status": "ok",
        "raw_speed": round(raw_speed, 2),
        "filtered_turn": round(filtered_turn, 2),
        "raw_gear": round(raw_gear, 2)
    })
#Heartbeat variables
last_heartbeat = 0
HEARTBEAT_TIMEOUT = 10 #second
watchdog_triggered = False


#access with token
@app.route("/session")
def new_session():
    import uuid
    token =str(uuid.uuid4())
    return {"token": token}

#Watchdog / Timeout Thread
def watchdog_loop():
    global last_heartbeat, watchdog_triggered, timeout_happened
    set_gpio = pigpio.pi()
    while True:
        time.sleep(1)
        if last_heartbeat == 0:
            continue
        #Check timeout
        time_to_reset = time.time() - last_heartbeat
        if time_to_reset < HEARTBEAT_TIMEOUT:
            print("Time to reset:", time_to_reset)
        else:
            print("Loss communication time:", time_to_reset)
        if time.time() - last_heartbeat > HEARTBEAT_TIMEOUT:
            if not watchdog_triggered:
                print("⚠️  Heartbeat timeout! Resetting values...")
                reset_all_values()
                watchdog_triggered = True
                timeout_happened = True
#Heartbeat Route
@app.route("/heartbeat")
def heartbeat():
    global last_heartbeat, watchdog_triggered, timeout_happened
    last_heartbeat = time.time()
    watchdog_triggered = False
    #return jsonify({"status": "OK"}), 200
    #print("timeout_happened:", timeout_happened)
    if timeout_happened:
        timeout_happened = False
        return {"status": "timeout"}, 408
    else:
        return jsonify({"status": "OK"}), 200
#Reset all servo pulses
def reset_all_values():
    set_gpio = pigpio.pi()
    print("🔄 Resetting...")
    set_gpio.set_servo_pulsewidth(SPEED_SERVO, 1500)
    set_gpio.set_servo_pulsewidth(STEERING_SERVO, 1500)
    set_gpio:set_servo_pulsewidth(GEAR_SERVO, 1500)
    set_gpio.stop()
@app.route('/stop')
def stop():
    pi.set_servo_pulsewidth(SPEED_SERVO, 1500)
    pi.set_servo_pulsewidth(STEERING_SERVO, 1500)
    pi:set_servo_pulsewidth(GEAR_SERVO, 1500)
    pi.stop()
    return "Servos stopped."

threading.Thread(target=watchdog_loop, daemon=True).start()

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
