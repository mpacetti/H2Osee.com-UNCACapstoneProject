#!/usr/bin/python3

# H2Osee.com
# Raspberry Pi Python Gateway Driver
#
#   Developer:  Mark Pacetti
#   Date:       09/18/19
#
# Continuously receives LoRa packets with sensor telemetry and publishes to MQTT broker
#
#
# =======================================================================================

import time
import datetime
import busio
import board
import adafruit_rfm9x
import paho.mqtt.client as mqtt
from digitalio import DigitalInOut, Direction, Pull

# Configure RFM9x LoRa radio on raspberry pi
CS = DigitalInOut(board.CE1)
RESET = DigitalInOut(board.D25)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

# LoRa client and server addresses
lora_client_address = 1
lora_server_address = 7

topic_air_temp = "US/NC/828/SEQWDSLK/Air Temperature @ 2ft"
topic_humidity = "US/NC/828/SEQWDSLK/Humidity"
topic_pressure = "US/NC/828/SEQWDSLK/Barometric Pressure"
topic_ground_temp = "US/NC/828/SEQWDSLK/Ground Temperature @ 1ft"
topic_water_temp_0ft = "US/NC/828/SEQWDSLK/Water Temperature @ 0ft"
topic_water_temp_2ft = "US/NC/828/SEQWDSLK/Water Temperature @ 2ft"
topic_altitude = "US/NC/828/SEQWDSLK/Altitude"
topic_uv_index = "US/NC/828/SEQWDSLK/UV Index"
topic_turbidity_ntu = "US/NC/828/SEQWDSLK/Turbidity NTU"
topic_tds = "US/NC/828/SEQWDSLK/TDS"
topic_ph = "US/NC/828/SEQWDSLK/pH"
topic_sats_tracked = "US/NC/828/SEQWDSLK/GPS Satellites Tracked"
topic_latitude = "US/NC/828/SEQWDSLK/Latitude"
topic_longitude = "US/NC/828/SEQWDSLK/Longitude"
topic_date = "US/NC/828/SEQWDSLK/UTC Date"
topic_time = "US/NC/828/SEQWDSLK/UTC Time"

# callback function to fire when successfully connected to broker
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("successfully connected to MQTT broker...")
        global Connected
        Connected = True
    else:
        print("connection to MQTT broker failed...")

# start out NOT connected
Connected = False
# mqtt server configuration
mqttbroker = "mqttbroker.h2osee.com"
port = 1883
keep_alive = 45

# create new mqtt client instance
client = mqtt.Client("h2osee_pi_python_driver")
# setup callback to on_connect
client.on_connect = on_connect
# connect to broker
client.connect(mqttbroker, port, keep_alive)
# start loop thread
client.loop_start()

while Connected is not True:
    time.sleep(0.1)

# Attempt to initialize the RFM9x module
try:
    rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, 915.0)
    print('RFM95 radio initialized successfully...')
except RuntimeError:
    print('RFM95 error...')


# Wait for LoRa packets and publish payload
try:
    while True:
        packet = None
        packet = rfm9x.receive()

        if packet is not None:
            print("received packet from RFM95 radio: ")
            print(str(packet))

            rfm9x.send(packet)
            from_addr = packet[0]
            to_addr = packet[1]
            if from_addr == lora_client_address and to_addr == lora_server_address:
                # parse the byte array
                air_temp = int.from_bytes(packet[2:4], byteorder='little') / 10.0
                humidity = int.from_bytes(packet[4:6], byteorder='little') / 10.0
                pressure = int.from_bytes(packet[6:8], byteorder='little') / 10.0
                ground_temp = int.from_bytes(packet[8:10], byteorder='little') / 10.0
                water_temp_0ft = int.from_bytes(packet[10:12], byteorder='little') / 10.0
                water_temp_2ft = int.from_bytes(packet[12:14], byteorder='little') / 10.0
                altitude = int.from_bytes(packet[14:16], byteorder='little') / 10.0
                uv_index = int.from_bytes(packet[16:18], byteorder='little')
                turbidity_ntu = int.from_bytes(packet[18:20], byteorder='little') / 10.0
                tds = int.from_bytes(packet[20:22], byteorder='little') / 10.0
                ph = int.from_bytes(packet[22:24], byteorder='little') / 10.0
                sats_tracked = packet[24]

                latitude = int.from_bytes(packet[25:27], byteorder='little', signed=True) / 100.0
                longitude = int.from_bytes(packet[27:29], byteorder='little', signed=True) / 100.0

                date_yyyy = int.from_bytes(packet[29:31], byteorder='little', signed=False)
                date_mm = packet[31]
                date_dd = packet[32]
                time_hh = packet[33]
                time_mm = packet[34]
                time_ss = packet[35]

                if date_mm <= 9:
                    date_mm = "0" + str(date_mm)
                else:
                    date_mm = str(date_mm)

                if date_dd <= 9:
                    date_dd = "0" + str(date_dd)
                else:
                    date_dd = str(date_dd)

                if time_mm <= 9:
                    time_mm = "0" + str(time_mm)
                else:
                    time_mm = str(time_mm)

                if time_ss <= 9:
                    time_ss = "0" + str(time_ss)
                else:
                    time_ss = str(time_ss)

                utc_date = str(date_yyyy) + "-" + date_mm + "-" + date_dd
                utc_time = str(time_hh) + ":" + time_mm + ":" + time_ss

                # Print results
                print("From: ", from_addr)
                print("To: ", to_addr)
                print("Air Temp: ", air_temp, "F")
                print("Humidity: ", humidity, "%")
                print("Pressure: ", pressure, "inHg")
                print("Ground Temp: ", ground_temp, "F")
                print("Water Temp @ 0ft: ", water_temp_0ft, "F")
                print("Water Temp @ 2ft: ", water_temp_2ft, "F")
                print("Altitude: ", altitude, "ft")
                print("UV Index: ", uv_index, "")
                print("Turbidity: ", turbidity_ntu, "NTU")
                print("TDS: ", tds, "ppm")
                print("pH: ", ph, "")
                print("Satellites Tracked: ", sats_tracked, "")
                print("Latitude:", latitude, "N")
                print("Longitude: ", longitude, "W")
                print("UTC Date:", utc_date, "")
                print("UTC Time:", utc_time, "")
                print()

                currentDT = datetime.datetime.now()
                print(str(currentDT))
                print()
                print()

                # publish sensor values to mqtt broker
                print("publishing messages to MQTT broker...")
                # publish values
                client.publish(topic_air_temp, air_temp, qos=2, retain=True)
                client.publish(topic_humidity, humidity, qos=2, retain=True)
                client.publish(topic_pressure, pressure, qos=2, retain=True)
                client.publish(topic_ground_temp, ground_temp, qos=2, retain=True)
                client.publish(topic_water_temp_0ft, water_temp_0ft, qos=2, retain=True)
                client.publish(topic_water_temp_2ft, water_temp_2ft, qos=2, retain=True)
                client.publish(topic_altitude, altitude, qos=2, retain=True)
                client.publish(topic_uv_index, uv_index, qos=2, retain=True)
                client.publish(topic_turbidity_ntu, turbidity_ntu, qos=2, retain=True)
                client.publish(topic_tds, tds, qos=2, retain=True)
                client.publish(topic_ph, ph, qos=2, retain=True)
                client.publish(topic_sats_tracked, sats_tracked, qos=2, retain=True)
                client.publish(topic_longitude, longitude, qos=2, retain=True)
                client.publish(topic_latitude, latitude, qos=2, retain=True)
                client.publish(topic_date, utc_date, qos=2, retain=True)
                client.publish(topic_time, utc_time, qos=2, retain=True)
                print("all messages published successfully...")
                print(20 * "|")
                
except KeyboardInterrupt:
    client.disconnect()
    client.loop_stop
