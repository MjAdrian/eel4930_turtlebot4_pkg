import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

# Define the GPIO pins for the servos
SERVO1_PIN = 17
SERVO2_PIN = 18

# Set up the GPIO pins as output
#GPIO.setup(SERVO1_PIN, GPIO.OUT)
GPIO.setup(SERVO2_PIN, GPIO.OUT)

# Set the PWM frequency
pwm_frequency = 50

# Initialize the PWM for each servo
#servo1 = GPIO.PWM(SERVO1_PIN, pwm_frequency)
servo2 = GPIO.PWM(SERVO2_PIN, pwm_frequency)

# Start the PWM for each servo with a 0% duty cycle
#servo1.start(0)
servo2.start(0)

try:
    while True:
        # Move the servos to different angles (0 to 180 degrees)
        for angle in range(0, 271, 10):
            duty_cycle = (angle / 180.0) * 10 + 2
            #servo1.ChangeDutyCycle(duty_cycle)
            servo2.ChangeDutyCycle(duty_cycle)
            print(angle)
            time.sleep(0.5)

        break

        # Move the servos back to the starting position (180 to 0 degrees)
        for angle in range(360, -1, -10):
            duty_cycle = (angle / 180.0) * 10 + 2
            #servo1.ChangeDutyCycle(duty_cycle)
            servo2.ChangeDutyCycle(duty_cycle)
            time.sleep(0.5)

    while True:
        duty_cycle = 2
        servo2.ChangeDutyCycle(duty_cycle)
        pass

except KeyboardInterrupt:
    # Stop the program when Ctrl+C is pressed
    print("Exiting...")

finally:
    # Clean up the GPIO pins and stop the PWM
    #servo1.stop()
    servo2.stop()
    GPIO.cleanup()

