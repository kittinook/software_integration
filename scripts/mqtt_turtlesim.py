#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int64
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist

import json
import logging
import random
import time
import math

from paho.mqtt import client as mqtt_client

from turtlesim_plus_interfaces.srv import GivePosition

BROKER = 'mqtt.netpie.io'
PORT = 1883
TOPIC = "@msg/command"
# generate client ID with pub prefix randomly
CLIENT_ID = 'e46cbbc7-60ed-4b3a-a36e-42f78e839b59'
USERNAME = 'LMmDGLSpeDSd3U5xjKZGhEJmEg73QpSd'
PASSWORD = 'b46EEunMf8wFtWwwu6ivi7QXDMKxWwKH'

FIRST_RECONNECT_DELAY = 1
RECONNECT_RATE = 2
MAX_RECONNECT_COUNT = 12
MAX_RECONNECT_DELAY = 60

FLAG_EXIT = False


class MQTTTurtlesim(Node):

    def __init__(self):
        super().__init__('mqtt_slave')
        self.cmdvel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(Int64, "turtle1/pizza_count", self.pizza_count_callback, 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.spawn_pizza_client = self.create_client(GivePosition, "spawn_pizza")
        self.eat_pizza_client = self.create_client(Empty, "turtle1/eat")
        self.i = 0
        self.client = self.connect_mqtt()
        self.client.loop_start()
        self.velocity = [0.0, 0.0]
        self.button_state = False
        self.isRecived = False
        self.lastTimeRecived = 0
        self.pizza_count = 0
        self.deadband = 0.1
        time.sleep(1)
        self.spawn_pizza([0.0, 0.0])

    def spawn_pizza(self, position):
        position_request = GivePosition.Request()
        position_request.x = position[0]
        position_request.y = position[1]
        self.spawn_pizza_client.call_async(position_request)

    def eat_pizza(self):
        eat_request = Empty.Request()
        self.eat_pizza_client.call_async(eat_request)

    def pizza_count_callback(self, msg):
        if self.pizza_count != msg.data:
            self.pizza_count = msg.data
            self.spawn_pizza([0.0, 0.0])

    def timer_callback(self):
        twist = Twist()
        if self.isRecived or time.time() - self.lastTimeRecived <= 2.0:
            if self.button_state == True:
                self.eat_pizza()
                self.button_state = False
            self.isRecived = False
            
            if math.sqrt(math.pow(self.velocity[0],2) + math.pow(self.velocity[1],2)) > self.deadband:
                twist.linear.x = self.velocity[0]
                twist.angular.z = self.velocity[1]
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            self.cmdvel_publisher.publish(twist)
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmdvel_publisher.publish(twist)
        # print(twist)

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0 and client.is_connected():
            print("Connected to MQTT Broker!")
            client.subscribe(TOPIC)
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
        
        json_buffer = json.loads(msg.payload.decode())
        self.lastTimeRecived = time.time()
        self.isRecived = True
        self.velocity[0] = json_buffer["velocity"][0]
        self.velocity[1] = json_buffer["velocity"][1]
        if self.button_state == False and json_buffer["button"] == True:
            self.button_state = True
        print(f'Received `{msg.payload.decode()}` from `{msg.topic}` topic --> dt `{time.time() - json_buffer["timestamp"]}` ')
            
        

    def connect_mqtt(self):
        client = mqtt_client.Client(CLIENT_ID)
        client.username_pw_set(USERNAME, PASSWORD)
        client.on_connect = self.on_connect
        client.on_message = self.on_message
        client.connect(BROKER, PORT, keepalive=120)
        client.on_disconnect = self.on_disconnect
        return client

    def publish(self, client):
        msg_count = 0
        while not FLAG_EXIT:
            msg_dict = {
                'msg': msg_count
            }
            msg = json.dumps(msg_dict)
            if not client.is_connected():
                logging.error("publish: MQTT client is not connected!")
                time.sleep(1)
                continue
            # result = client.publish(TOPIC, msg)
            # result: [0, 1]
            status = 0#result[0]
            if status == 0:
                print(f'Send `{msg}` to topic `{TOPIC}`')
            else:
                print(f'Failed to send message to topic {TOPIC}')
            msg_count += 1
            time.sleep(1)



def main(args=None):
    rclpy.init(args=args)

    mqtt_turtlesim = MQTTTurtlesim()

    rclpy.spin(mqtt_turtlesim)
    mqtt_turtlesim.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
