import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

# Define the GPIO pins for the servos
SERVO1_PIN = 17
SERVO2_PIN = 18

# Set up the GPIO pins as output
GPIO.setup(SERVO1_PIN, GPIO.OUT)
# GPIO.setup(SERVO2_PIN, GPIO.OUT)

# Set the PWM frequency
pwm_frequency = 50

# Initialize the PWM for each servo
servo1 = GPIO.PWM(SERVO1_PIN, pwm_frequency)
# servo2 = GPIO.PWM(SERVO2_PIN, pwm_frequency)

# Start the PWM for each servo with a 0% duty cycle
servo1.start(0)
# servo2.start(5)

try:
    while True:
        # Control the continuous rotation servo with different speeds and directions
        # Stop
        servo1.ChangeDutyCycle(7.5)  # 1.5 ms pulse width
        time.sleep(2)

        # Rotate clockwise at a certain speed
        servo1.ChangeDutyCycle(10)  # Increase the duty cycle to make it rotate faster
        time.sleep(2)

        # Rotate counterclockwise at a certain speed
        servo1.ChangeDutyCycle(5)  # Decrease the duty cycle to make it rotate in the opposite direction
        time.sleep(2)

        # # Control the regular servo with different angles (0 to 180 degrees)
        # for angle in range(0, 181, 10):
        #     duty_cycle = (angle / 180.0) * 10 + 2
        #     servo2.ChangeDutyCycle(duty_cycle)
        #     time.sleep(0.5)

        # # Move the regular servo back to the starting position (180 to 0 degrees)
        # for angle in range(180, -1, -10):
        #     duty_cycle = (angle / 180.0) * 10 + 2
        #     servo2.ChangeDutyCycle(duty_cycle)
        #     time.sleep(0.5)

except KeyboardInterrupt:
    # Stop the program when Ctrl+C is pressed
    print("Exiting...")

finally:
    # Clean up the GPIO pins and stop the PWM
    servo1.stop()
    # servo2.stop()
    GPIO.cleanup()
