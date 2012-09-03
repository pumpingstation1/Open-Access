#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  UserSync.py
#  
#  Copyright 2012 Rhys Rhaven <rhys@rhavenindustrys.com>
#  
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 3 of the License, or
#  (at your option) any later version.
#  
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#  

import json
import serial
import string
from pprint import pprint

dump_size = 200									# Number of lines output by the userdump
serial_port_speed = 57600						# Speed of the Arduino serial connection
console_password = "1234"						# Password to enable privledged mode on the console

def main():

	user_list_file = open("userList.json", "rt")			# Load File Database
	user_list = json.load(user_list_file)				
	
	user_list_embed = getUserList('/dev/ttyUSB0')			# Load Actual Database
	
	tag_dict = {}											# Create a dictionary from the actual database
	for item in user_list_embed:							# So we can lookup entries based on the info in the file db
		tag_dict[item['tag_value']] = item['eeprom_pos']	# We don't want to store eeprom position in the file db
		
	print tag_dict
	return 0
	
def getUserList(serial_port):
# Opens serial port to Open Access Controller
# Gets embedded user db, converts to JSON object
# Returns JSON Object
	
	s_port = serial.Serial(serial_port, serial_port_speed, timeout=1)
	json_dump = ""
	dump_list = []
	
	s_port.write("e " + console_password + "\r")
	if s_port.readline() == "Privileged mode enabled.":
		print "Console password incorrect, exiting."
		exit(1)
	
	s_port.write("a\r")
	
	while True:
		dump_list.append(s_port.readline())
		if len(dump_list) >= dump_size:
			break
	
	for item in dump_list:
		json_dump += item.rstrip()
	
	json_dump_parsed = json.loads(json_dump)
	
	return json_dump_parsed
	

if __name__ == '__main__':
	main()

