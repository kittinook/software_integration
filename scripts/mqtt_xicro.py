#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3

import json
import logging
import random
import time
import math

from paho.mqtt import client as mqtt_client

BROKER = 'mqtt.netpie.io'
PORT = 1883
TOPIC = "@msg/command"
# generate client ID with pub prefix randomly
CLIENT_ID = '31c43089-e299-457a-a79f-468df0ff67d3'
USERNAME = 'DhcqTysDQBhLipuqiYxxjnAz5ceg7dKt'
PASSWORD = 'w1uuTkaj4HGWg8uhcaVtm8WgCRJsUgAy'

FIRST_RECONNECT_DELAY = 1
RECONNECT_RATE = 2
MAX_RECONNECT_COUNT = 12
MAX_RECONNECT_DELAY = 60

FLAG_EXIT = False


class MQTTXicro(Node):

    def __init__(self):
        super().__init__('mqtt_slave')
        self.cmdvel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.create_subscription(Vector3,"raw_accel" ,self.acc_callback, 10)
        self.i = 0
        self.client = self.connect_mqtt()
        self.client.loop_start()
        self.acc_imu = Vector3()
        self.k_v = 0.03
        self.k_w = 0.01
        self.accelerationZ = 0
        self.g = 9.81
        self.button_state = False
        self.pitch = 0
        self.roll = 0

    def acc_callback(self, msg):
        self.acc_imu = msg

    def timer_callback(self):
        accelerationX = self.acc_imu.x
        accelerationY = self.acc_imu.y
        accelerationZ = self.acc_imu.z
        if abs(self.accelerationZ - accelerationZ) >= 2*self.g:
            self.button_state = True

        if accelerationX != 0 and accelerationY != 0 and accelerationZ != 0:
            
            self.pitch = 180 * math.atan (accelerationX/math.sqrt(accelerationY*accelerationY + accelerationZ*accelerationZ))/math.pi
            self.roll = 180 * math.atan (accelerationY/math.sqrt(accelerationX*accelerationX + accelerationZ*accelerationZ))/math.pi
            # print(str(pitch) + "\t" + str(roll))
        v_x = self.pitch * self.k_v
        w = self.roll * self.k_w
        command = {
            "velocity" : [v_x, w],
            "button" : self.button_state,
            "timestamp" : time.time()
        }
        print(command)
        buffer = json.dumps( command )
        self.client.publish(TOPIC, buffer)
        self.button_state = False

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0 and client.is_connected():
            print("Connected to MQTT Broker!")
            # client.subscribe(TOPIC)
        else:
            print(f'Failed to connect, return code {rc}')

    def on_disconnect(self, client, userdata, rc):
        logging.info("Disconnected with result code: %s", rc)
        reconnect_count, reconnect_delay = 0, FIRST_RECONNECT_DELAY
        while reconnect_count < MAX_RECONNECT_COUNT:
            logging.info("Reconnecting in %d seconds...", reconnect_delay)
            time.sleep(reconnect_delay)

            try:
                client.reconnect()
                logging.info("Reconnected successfully!")
                return
            except Exception as err:
                logging.error("%s. Reconnect failed. Retrying...", err)

            reconnect_delay *= RECONNECT_RATE
            reconnect_delay = min(reconnect_delay, MAX_RECONNECT_DELAY)
            reconnect_count += 1
        logging.info("Reconnect failed after %s attempts. Exiting...", reconnect_count)
        global FLAG_EXIT
        FLAG_EXIT = True

    def on_message(self, client, userdata, msg):
        print(f'Received `{msg.payload.decode()}` from `{msg.topic}` topic')

    def connect_mqtt(self):
        client = mqtt_client.Client(CLIENT_ID)
        client.username_pw_set(USERNAME, PASSWORD)
        client.on_connect = self.on_connect
        client.on_message = self.on_message
        client.connect(BROKER, PORT, keepalive=120)
        client.on_disconnect = self.on_disconnect
        return client



def main(args=None):
    rclpy.init(args=args)

    mqtt_xicro = MQTTXicro()

    rclpy.spin(mqtt_xicro)
    mqtt_xicro.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
