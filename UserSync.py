#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  untitled.py
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
from pprint import pprint

def main():
	
	# Write a file
	user_list_file = open("userList.json", "rt")
	user_list = json.load(user_list_file)
	
	# pprint(userList[0])
	sport = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)
	dump_list = list()
	
	sport.write("a\r")
	json_dump = ""
	while True:
		dump_list.append(sport.readline())
		if len(dump_list) == 200:
			break
	
	for item in dump_list:
		json_dump += item.rstrip()
	
	#print json_dump	
	json_dump_parsed = json.loads(json_dump)
	pprint(json_dump_parsed)
	#print json.dumps(user_list)
	
	return 0

if __name__ == '__main__':
	main()

