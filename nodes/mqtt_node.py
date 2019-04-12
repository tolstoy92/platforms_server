#!/usr/bin/env python


import rospy
import paho.mqtt.client as mqtt
from mqtt_utils.mqtt_utils import MqttClientTools
from platforms_server.msg import FieldObjects

SERVER_IP = rospy.get_param('SERVER_IP')
PORT = rospy.get_param('PORT')
MESSAGES_QOS = rospy.get_param('MESSAGES_QOS')
CONNECTON_TOPIC = rospy.get_param('CONNECTON_TOPIC')
FEEDBACK_TOPIC = rospy.get_param('FEEDBACK_TOPIC')
DELAY_TIME = rospy.get_param('DELAY_TIME')
WORKER_TIME = rospy.get_param('WORKER_TIME')
MAIN_TOPIC = rospy.get_param('MAIN_TOPIC')

client = mqtt.Client("Server")
msg_sender = MqttClientTools(SERVER_IP, PORT, MESSAGES_QOS, CONNECTON_TOPIC, DELAY_TIME)

msg_sender.start_connection_client(client, )
client.subscribe(FEEDBACK_TOPIC, qos=0)
client.loop(WORKER_TIME)

delays = {}


def mqtt_callback(msg_data):
    global delays
    for robot in msg_data.robots:
        if robot.path_created:
            platform_topic = MAIN_TOPIC + str(robot.id)
            angle_msg = msg_sender.prepare_angle_msg(robot.actual_angle)
            move_msg = msg_sender.prepare_move_msg(robot.move)
            rotate_msg = msg_sender.prepare_rotation(robot.rotation)
            finish_status = robot.on_finish_point
            finish_msg = msg_sender.prepare_finish_msg(finish_status)
            final_msg = angle_msg + move_msg + rotate_msg + finish_msg
            if robot.id not in delays.keys():
                delays[robot.id] = 0
            msg_sender.delay_time = 0.2
            delays = msg_sender.send_msg_with_delay(delays, robot.id, client,
                                                    platform_topic, final_msg)


rospy.init_node("mqtt_node")
fields_data_sub = rospy.Subscriber("field_objects", FieldObjects, mqtt_callback)
rospy.spin()
