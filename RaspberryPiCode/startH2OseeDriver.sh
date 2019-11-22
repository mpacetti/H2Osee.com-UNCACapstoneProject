#!/bin/sh

ps -FC python3  | grep PiGatewayDriver.py | grep -v grep > /dev/null

if [ $? != 0 ]
then
	/usr/bin/python3 -u /home/pi/h2osee/PiGatewayDriver.py > /home/pi/h2osee/driver.log &
fi
