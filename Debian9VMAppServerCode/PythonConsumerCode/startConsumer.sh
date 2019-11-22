#!/bin/sh

ps -FC python3  | grep CloudMQTTConsumer.py | grep -v grep > /dev/null

if [ $? != 0 ]
then
	/usr/bin/python3 -u /home/markpacetti/h2osee/CloudMQTTConsumer.py > /home/markpacetti/h2osee/driver.log &
fi
f