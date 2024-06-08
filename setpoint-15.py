import RPi.GPIO as GPIO
from gpiozero import Servo, MCP3008
from time import sleep, time

# Pin setup
GPIO_TRIGGER = 23
GPIO_ECHO = 24
SERVO_PIN = 18
#pot = MCP3008(channel=0)

# Constants
START_ANGLE = 50  # Initial angle for the servo
SERVO_MIN_ANGLE = 0
SERVO_MAX_ANGLE = 120
SETPOINT = 15  # Desired point in cm
Kp = 5  # Proportional constant
Ki = 0.05 # Integration constant
Kd = 0.2  # Derivation constant

# PID variables
pError = 0
iError = 0
dError = 0
prevError = 0
prevTime = time()

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

# Setup servo
servo = Servo(SERVO_PIN, min_pulse_width=0.001, max_pulse_width=0.002)
servo.value = (START_ANGLE / 120.0) * 2 - 1

#def get_setpoint():
   # if 0<pot.value<0.33:
      #  SETPOINT = 5
    #elif 0.33<pot.value<0.66:
      #  SETPOINT = 15
    # elif 0.66<pot.value<1:
    #    SETPOINT = 25
    
    #return SETPOINT
    
    

def get_distance():
    # Trigger'ı HIGH yap
    GPIO.output(GPIO_TRIGGER, True)

    # 0.01ms sonra LOW yap
    sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    StartTime = time()
    StopTime = time()

    # StartTime'ı kaydet
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time()

    # StopTime'ı kaydet
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time()

    # Start ve Stop arasındaki zaman farkını hesapla
    TimeElapsed = StopTime - StartTime
    # Mesafeyi hesapla
    distance = (TimeElapsed * 34300) / 2

    return distance

def constrain_servo_angle(distance, angle):
    if distance > 15 and angle > 50:
        return 50
    elif distance < 15 and angle < 50:
        return 50
    return angle

try:
    while True:
        distance = get_distance()
        
        # Filter out invalid distances
        if distance > 35:
            distance = 35  # Ignore out-of-range values
        
        # PID calculations
        now = time()
        dt = now - prevTime
        pError = SETPOINT - distance
        dError = (pError - prevError) / dt
        iError += pError * dt
        
        # Limit the integral term to prevent windup
        if iError > 10:
            iError = 10
        elif iError < -10:
            iError = -10
        
        output = Kp * pError + Ki * iError + Kd * dError
        
        # Limit the output
        if output > 50:
            output = 50
        elif output < -50:
            output = -50
        
        # Calculate the new servo position
        servo_angle = START_ANGLE + output
        if servo_angle < SERVO_MIN_ANGLE:
            servo_angle = SERVO_MIN_ANGLE
        elif servo_angle > SERVO_MAX_ANGLE:
            servo_angle = SERVO_MAX_ANGLE
        
        # Constrain the servo angle based on distance
        servo_angle = constrain_servo_angle(distance, servo_angle)
        
        # Set the servo position
        servo.value = (servo_angle / 120.0) * 2 - 1
        
        # Print PID output for debugging
        print(f"PID: {output}, Distance: {distance} cm, Servo angle: {servo_angle}")
        
        # Update previous error and time
        prevError = pError
        prevTime = now
        
        # Delay to allow the loop to run at a reasonable speed
        sleep(0.1)

except KeyboardInterrupt:
    print("Measurement stopped by user")
    GPIO.cleanup()
