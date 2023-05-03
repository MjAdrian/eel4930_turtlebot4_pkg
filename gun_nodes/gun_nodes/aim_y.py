import rclpy
from rclpy.node import Node
from autoro_interfaces.srv import YAng

import math
import RPi.GPIO as GPIO
import time


class Aim_Y(Node):
    """Aims gun up and down according to angle value requested by client"""

    def __init__(self):
        super().__init__("aim_y")

        self.srv = self.create_service(YAng, 'Aim_Y', self.callback)

        self.cur_ang = 0

        GPIO.setmode(GPIO.BCM)

        # Define the GPIO pins for the servos

        SERVO1_PIN = 17     # Angle

        GPIO.setup(SERVO1_PIN, GPIO.OUT)

        pwm_frequency = 50


        self.ang_serv = GPIO.PWM(SERVO1_PIN, pwm_frequency)

        self.ang_serv.start(0)

    def callback(self, req, resp):
        if(req.angle > 0):
            duty = 3.75
        else:
            duty = 11.25
        t = (abs(req.angle)/360.0)*(179/19)*.75

        self.ang_serv.ChangeDutyCycle(duty)
        time.sleep(t)
        self.ang_serv.ChangeDutyCycle(0)
        self.cur_ang = self.cur_ang + req.angle

        resp.cur_ang = self.cur_ang

        return resp

   
def main(args=None):
    rclpy.init(args=args)
    aim = Aim_Y()
    rclpy.spin(aim)
    aim.ang_serv.stop()
    aim.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()