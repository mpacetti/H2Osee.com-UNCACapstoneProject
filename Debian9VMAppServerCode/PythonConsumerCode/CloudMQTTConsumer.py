#!/usr/bin/python3

# H2Osee.com
# Google Cloud Platform App Server VM MQTT Consumer
#
# MARK PACETTI
#
# Continuously received subscribed MQTT messages and inserts them into MySQL database table 'sample'
#
# Oct 1, 2019
#
# =======================================================================================

import paho.mqtt.client as mqtt
import mysql.connector
from mysql.connector import Error

mqttbroker = "mqttbroker.h2osee.com"
mqttport = 1883
mqttkeepalive = 60

# GCP Cloud SQL - MySQL server internal address (must be local or use proxy to connect)
dbserver = "35.245.229.164"
dbname = "h2oseedb"
dbuser = "dbuser"
dbpassword = "DBpassword2019!"
dbtable = "sample"


# function to consume message and perist
def insertMQTTmessage(full_topic, msg_payload):
    try:
        topic_list = full_topic.split("/")
        topic_base = topic_list[0] + "/" + topic_list[1] + "/" + topic_list[2]
        topic_location = topic_list[3]
        topic_sensor = topic_list[4]
        reading = msg_payload

        insert_cmd = """INSERT INTO sample (topicbase, locationid, sensor, reading) VALUES (%s, %s, %s, %s) """
        insert_cols = (topic_base, topic_location, topic_sensor, reading)

        connection = mysql.connector.connect(host=dbserver,
                                             database=dbname,
                                             user=dbuser,
                                             password=dbpassword)
        if connection.is_connected():
            cursor = connection.cursor(prepared=True)
            result = cursor.execute(insert_cmd, insert_cols)
            connection.commit()

    except mysql.connector.Error as e:
        print("failed to insert mqtt message {}".format(e))

    finally:
        if connection.is_connected():
            cursor.close()
            connection.close()


# callback for when the listener receives CONNACK response from the MQTT broker
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        client.connected_flag = True
        # insures if connection is lost the subscription will be renewed
        client.subscribe("#", 2)
        print("connected to MQTT broker successfully and subscribed to topics...")
    else:
        print("bad connection to MQTT broker, return code: ", rc)


# the callback function when a PUBLISH message is received from the MQTT broker
def on_message(client, userdata, msg):
    print(msg.topic + " -> " + msg.payload.decode("utf-8"))
    # store to SQL db
    insertMQTTmessage(msg.topic, msg.payload.decode("utf-8"))


# connect to MQTT client object and define callbacks
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message


client.connect(mqttbroker, mqttport, mqttkeepalive)
print("successfully connected to <" + mqttbroker + "> MQTT broker...")

# blocking call processes network traffic, dispatches callbacks and handles reconnecting
client.loop_forever()
