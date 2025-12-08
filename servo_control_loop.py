import time
import csv
from kalman_filter import KalmanFilter
import pigpio
from map_degree_pwm import map_value
# Initialize pigpio
pi = pigpio.pi()

csv_file = open("steering_log.csv", "a", newline="")
writer = csv.writer(csv_file)
writer.writerow(["target_steering_value", "kalman_steering_value"])

# function of setting the steering value until reaching setting value.
def servo_control(servo_motor, target_value):
    """
    target_value: the final desired servo pulse width (e.g., 1700)
    kalman_filter: function that takes x and returns filtered y
    set_servo_func: function that actually moves the servo to y
    """
    while True:
	# set mode gpio
        pi.set_mode(servo_motor, pigpio.OUTPUT)
        # → run Kalman filter
        temp_target_value = target_value #Help to track the steering_log.csv
        kalmal_process = KalmanFilter(process_variance=0.01, measurement_variance=0.1)
        kalman_target_value = kalmal_process.update(target_value)
        pwm_value =  map_value(kalman_target_value, -45, 45, 1000, 2000)
        #print(f"Kalman_target_value={target_value}, output={kalman_target_value}")
        print(f"Round of target value kalman: {kalman_target_value}")
        # → set servo position
        pi.set_servo_pulsewidth(servo_motor, pwm_value)
        print(f"Servo set to {kalman_target_value}")
        writer.writerow([int(temp_target_value), int(kalman_target_value)])
        csv_file.flush()

        # → check stop condition
        if int(kalman_target_value) == int(target_value):
            print("Target reached. Stopping loop.")
            break

        # → update for next round
        target_value = kalman_target_value

        # → wait 10 ms
        time.sleep(0.01)
