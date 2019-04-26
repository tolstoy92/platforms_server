#!/usr/bin/env python


import rospy
import paho.mqtt.client as mqtt
from mqtt_utils.mqtt_utils import MqttClientTools
from platforms_server.msg import FieldObjects, IK_Data

SERVER_IP = rospy.get_param('SERVER_IP')
PORT = rospy.get_param('PORT')
MESSAGES_QOS = rospy.get_param('MESSAGES_QOS')
CONNECTON_TOPIC = rospy.get_param('CONNECTON_TOPIC')
FEEDBACK_TOPIC = rospy.get_param('FEEDBACK_TOPIC')
DELAY_TIME = rospy.get_param('DELAY_TIME')
WORKER_TIME = rospy.get_param('WORKER_TIME')
MAIN_TOPIC = rospy.get_param('MAIN_TOPIC')


def mqtt_reciever_callback(client, userdata, message):
    if message.topic[:7] == 'ik_data':
        data = message.topic.split('/')
        robot_id = int(data[1])
        robot_side = int(data[2])
        mqtt_msg = int(message.payload.decode("utf-8"))
        msg = IK_Data()
        msg.robot_id = robot_id
        msg.robot_side = robot_side
        msg.ik_data = mqtt_msg
        ik_data_pub.publish(msg)


def mqtt_transmitter_callback(msg_data):
    global delays
    for robot in msg_data.robots:
        # if robot.path_created:
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
ik_data_pub = rospy.Publisher("ik_data", IK_Data, queue_size=1)

client = mqtt.Client("Server")
msg_sender = MqttClientTools(SERVER_IP, PORT, MESSAGES_QOS, CONNECTON_TOPIC, DELAY_TIME, ik_data_pub)
msg_sender.start_connection_client(client, )
client.subscribe('ik_data/#', qos=0)
client.on_message = mqtt_reciever_callback
client.loop(WORKER_TIME)
delays = {}

fields_data_sub = rospy.Subscriber("field_objects", FieldObjects, mqtt_transmitter_callback)

rospy.spin()
