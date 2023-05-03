import rclpy
from rclpy.node import Node
from autoro_interfaces.srv import Trig

import math
import RPi.GPIO as GPIO
import time


class Shoot(Node):
    """Aims gun up and down according to angle value requested by client"""

    def __init__(self):
        super().__init__("shoot")

        self.srv = self.create_service(Trig, 'Trig', self.callback)

        GPIO.setmode(GPIO.BCM)

        # Define the GPIO pins for the servos

        SERVO2_PIN = 18     # Angle

        GPIO.setup(SERVO2_PIN, GPIO.OUT)

        pwm_frequency = 50


        self.trig_serv = GPIO.PWM(SERVO2_PIN, pwm_frequency)

        self.trig_serv.start(5)
        self.trig_serv.ChangeDutyCycle(0)

    def callback(self, req, resp):
        press_trigger = (60.0 / 180.0) * 10.0

        self.trig_serv.ChangeDutyCycle(press_trigger)
        time.sleep(req.time)
    
        self.trig_serv.ChangeDutyCycle((93/180)*10)
        time.sleep(.25)
        self.trig_serv.ChangeDutyCycle(0)

        resp.success = True

        return resp

   
def main(args=None):
    rclpy.init(args=args)
    shoot = Shoot()
    rclpy.spin(shoot)
    shoot.trig_serv.ChangeDutyCycle((93/180)*10)
    shoot.trig_serv.stop()
    shoot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()