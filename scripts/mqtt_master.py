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


class MQTTMaster(Node):

    def __init__(self):
        super().__init__('mqtt_master')
        self.cmdvel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.2  # seconds
        self.create_subscription(String,"pub2mqtt" ,self.pub2mqtt_callback, 10)
        self.client = self.connect_mqtt()
        self.client.loop_start()

    def pub2mqtt_callback(self, msg):
        command = {
            "message" : msg.data,
            "timestamp" : time.time()
        }
        print(command)
        buffer = json.dumps( command )
        self.client.publish(TOPIC, buffer)

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

    mqtt_master = MQTTMaster()

    rclpy.spin(mqtt_master)
    mqtt_master.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
