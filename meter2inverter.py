#!/usr/bin/python3

"""
OLED SSD1306
pip3 install adafruit-circuitpython-ssd1306
not found if running as root .  run systemctl as pi


https://learn.adafruit.com/nokia-5110-3310-monochrome-lcd/python-setup
pip3 install adafruit-circuitpython-pcd8544


sudo apt-get install python3-pil
sudo apt-get install python3-numpy
pip3 install RPi.GPIO
"""

version = 'version 1.3'

import os
import logging
root = '/home/pi/meter2inverter/'
# debug, info, warning, error, critical. default warning
log_file = root + "meter2inverter.log"
print ("logging to:  " , log_file)

if os.path.exists(log_file) == False:
    print("creating log file ", log_file)
    with open(log_file, "wb") as f:
        pass

### WARNING configure log file before any module which may import logging
#logging.warning('starting')  BEFORE .basicConfig will lof to terminal, and subcsequent .basicConfig will NOT change this*
# # Be sure to try the following in a newly started Python interpreter, and don’t just continue from the session described above:

logging.basicConfig(filename=log_file, format='%(asctime)s %(levelname)s %(message)s', level=logging.INFO)


from time import sleep, time
import datetime
import serial
import subprocess
import sys
import socket
#import select

import _thread # python3 LOW level threading
#import signal

from uuid import getnode

#from subprocess import call
#from subprocess import check_output

from random import randrange

# The httplib module has been renamed to http.client in Python 3.0.
# This module defines classes which implement the client side of the HTTP and HTTPS protocols. It is normally not used directly — 
# the module urllib uses it to handle URLs that use HTTP and HTTPS.
import http.client as httplib  # httplib only P2
import urllib

import board
import busio
import digitalio
#import adafruit_pcd8544 # Nokia lcd low res
#import adafruit_ssd1306 # use oled text instead. easier to rewrite same line

from PIL import Image, ImageDraw, ImageFont

#https://pypi.org/project/oled-text/
#pip3 install oled-text
# CAN USE LAYOUT, ICON, FONTS
# simpler, rewrite line
from oled_text import OledText

import RPi.GPIO as GPIO


########
# key settings
########

reboot = 18 #pull up. connect to GND to reboot or halt
led_traffic = 23 #  gpio out yellow
# pulse at app start
# pulse at relay start
pulse_led = 0.0005
# quick pulse traffic led in sec; every receive from either serial or socket 
# small value otherwize GX see EM disconnected

led_alarm = 27 #  gpio out red
# quick pulse alarm led at every valid pong received

modulo = 50 # print stat on oled every modulo frame received

ping_frequency = 60 # send ping on keep alive every so often
# make sure timeout on received in set accordingly


#############
# timeout on read
############

### MODBUS
# 11 bits , 9600 bits/sec
# > 3.5 char between 2 frames
# < 1.5 char between 2 char
# 1,14 ms per char
# max 1,72 ms between char
# min 4ms between frames

# in sec
timeout_serial = 0.002

#timeout_serial = 0.001

# avoid returning too early with partial modbus frames
nb_serial =  1 # will return when received that many, or whatever at timeout

nb_socket=10
timeout_socket = 0.001


#############
# application specific
#############

import secret
import monit_mail
import pushover
import arg

try:
	arg = arg.parse_arg()
	print(arg)
except Exception as e:
	print('exception arg parse ', str(e))
	# when running with vscode remote container from DEEP, launch.json is in DEEP/.vscode
	sys.exit(1)


#################
####### functions
#################

def signal_handler(sig, frame):
	print('You pressed Ctrl+C!')
	clean()
	sys.exit(0)


def exit_main():
	s = "exit main"
	logging.error(s)
	print(s)

	clean()

	sleep(1)

	print("SYSSSSSS EXITTTTTT")
	sys.exit(1)



# process clean up
# close connection
# set global to stop thread from main or main from thread
# set GPIO

def clean():
	global stop_thread
	global stop_main

	s = 'clean up: close connection. set global, GPIO, OLED'
	logging.error(s)
	print(s)


	# setting globals and closing connection could have been done already. does not eat bread

	stop_thread = True
	stop_main = True

	try:
		con.close()
	except:
		pass

	try:
		kas_con.close()
	except:
		pass

	try:
		kac.close()
	except:
		pass

	# could be executed multiple time, main and threads
	GPIO.output(led_alarm,0) 
	GPIO.output(led_traffic,0)

	oled_text("          ",1)
	oled_text("          ",2) 


def exit_thread(sock):
	global stop_main

	# ask main to exit
	stop_main = True

	s = "exit thread"
	logging.error(s)
	print(s)

	try:
		sock.close()
	except:
		pass

	clean()

	sleep(1)

	_thread.exit()


# polling
def check_reboot():
	if GPIO.input(reboot) == 0:
		s = 'halt/reboot button pressed' 
		logging.info(s)
		print(s)

		oled_text('halt/reboot', 1)

		sleep(2) # sleeping in isr is not great, but will reboot or halt anyway

		# long press = reboot
		if GPIO.input(reboot) == 0:
			s = 'reboot button pressed' 
			logging.info(s)
			oled_text('reboot', 1)
			clean() # to set gpio to off
			os.system('sudo reboot')
		else:

			s = 'halt button pressed' 
			logging.info(s)
			oled_text('halt', 1)
			clean() # to set gpio to off
			os.system('sudo halt')




# MAC address from getnode()
#pi_gx = 66035304507 # zero with ftdi. usb
pi_gx =  969326350912 # zero with ftdi. USB antenna
pi_em = 202481590087321 # zero W

ip_gx = "192.168.1.201"
ip_em = "192.168.1.202"
port_ip = 5000

mac = getnode()
print('MAC: ', mac)
role = None
if mac == pi_em:
	print('>>>>>>> running as energy meter')
	role = 'em'
	remote_ip= ip_gx # 

elif mac == pi_gx:
	print('>>>>>>> running as gx')
	role = 'gx'
	remote_ip= ip_em # of EM
else:
	s= 'unknown node. exit'
	print(s)
	logging.error(s)
	clean()
	sys.exit(1)


s = 'start meter2inverter %s' %version
print(s)
logging.info(s)
pushover.send_pushover(message='%s meter2inverter: %s '%(role,s), title='starting', priority=0)


#########
# set GPIO
#########

# ISR
def int_reboot(x):
	check_reboot()


GPIO.setmode(GPIO.BCM)
GPIO.setup(reboot, GPIO.IN, pull_up_down=GPIO.PUD_UP)
# use interrupt
GPIO.add_event_detect(reboot, GPIO.FALLING, callback=int_reboot, bouncetime=300)  

GPIO.setup(led_traffic, GPIO.OUT)
GPIO.setup(led_alarm, GPIO.OUT)

# in case PCB trace touches SDA
GPIO.setup(4, GPIO.IN)

GPIO.output(led_traffic,0) # turn off 
GPIO.output(led_alarm,1) # turn on. will turn off if connect is OK

# catch control C
#signal.signal(signal.SIGINT, signal_handler)

def blink_led(led,sec):
	GPIO.output(led,1)
	sleep(sec)
	GPIO.output(led,0)
	sleep(sec)



##########
# I2C OLED
##########

def oled_text(s,l):
	if is_display:
		oled.text(s,l)


i2c = busio.I2C(board.SCL, board.SDA)
print('i2c created ', i2c)

try:
	# 32 : larger font on 128x64
	oled = OledText(i2c, 128, 32) # 32 only line 1 to 3. line 4 is exception
	is_display = True
except Exception as e:
	s = 'cannot create oled text %s'  % str(e)
	logging.error(s)
	is_display = False
	print(s)

oled_text('>>%s starting' %role, 1)




####################
# keep alive server 
# if cannot bind after n try, exit
# wait (accept) FOREVER for a connection
# endless loop
# recev 4 bytes with timeout
# on recv exception or b'', exit
# send pong
# exit is asked by global
###################

def keep_alive_server(x):
	global stop_thread
	global stop_main

	s = 'KAS: start on port: %d' %(port_ip+1)
	logging.info(s)
	print(s)

	kas = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	for i in range(5):
		try:
			s = 'KAS: trying %d to bind on port %d' %(i,port_ip+1)
			logging.info(s)
			print(s)

			kas.bind(("", port_ip+1)) # same port +1

			s = 'KAS: bound %s' %str(kas)
			logging.info(s)
			print(s)

			break

		except Exception as e:

			# Binding Socket: "Address already in use"
			if i < 4:
				sleep(15)
				#kas.close() # instead of SO_REUSADDR. because issue with TIMEWAIT
				#kas.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
			else:
				s = 'KAS: %d cannot bind  %d %s' %(i, port_ip+1, str(e))
				logging.error(s)
				print(s)
				pushover.send_pushover(message='%s meter2inverter: %s '%(role,s), title='exception bind', priority=1)

				oled_text('KAS: BIND ERROR',2) # line
				oled_text('EXIT',1) # line

				# exit from own will. also tell main to sys.exit
				exit_thread(kas)

	# for i in range

	s = 'KAS: listening' 
	logging.debug(s)
	print(s)

	kas.listen()

	s = 'KAS: waiting for connection' 
	logging.debug(s)
	print(s)

	kas_con, addr = kas.accept() # use con to send/rec
	s = 'KAS: connected %s' %str(addr) 
	logging.debug(s)
	print(s)

	# non blocking and return b'' ?? so set timeout
	kas_con.settimeout(2*ping_frequency + 5) # for all operations. make sure this relates to ping frequency 
	# larger than ping , or will get timeout

	# wait forever for ping, send immediatly pong

	while True:
		try:

			b = kas_con.recv(4) # receive pong. 
			# The maximum amount of data to be received at once is specified by bufsize.
			# if remote killed, seems non blocking anymore and receive b''

			if b != b'':
				try:
					kas_con.sendall(b)

				except Exception as e:
					s = 'KAS: send ERROR' 
					oled_text('PONG ERROR',2) # line
					oled_text('EXIT',1) # line

					s = s + str(e)
					logging.error(s)
					print(s)
					pushover.send_pushover(message='%s meter2inverter: %s '%(role, role + s), title='exception send PONG', priority=1)

					exit_thread(kas_con)

			else:
				s = 'KAS: recv EMPTY' 
				# if remote killed, seems non blocking anymore and receive b''
				oled_text(s,2) # line
				oled_text('EXIT',1) # line

				logging.error(s)
				print(s)
				pushover.send_pushover(message='%s meter2inverter: %s '%(role,s), title='exception recv EMPTY', priority=1)

				exit_thread(kas_con)

		except Exception as e:
				s = 'KAS: recv EXCP'
				oled_text(s,1) # line
				oled_text('EXIT',2) # line

				s = s + str(e)
				logging.error(s)
				print(s)
				pushover.send_pushover(message='%s meter2inverter: %s '%(role,s), title='exception KAS recv', priority=1)

				exit_thread(kas_con)

	if stop_thread == True:
		# global set by main
		print("KAS: stop_thread", stop_thread)

		s = 'KAS: must STOP'
		oled_text(s,1) # line
		oled_text('EXIT',2) # line

		logging.error(s)
		print(s)
		pushover.send_pushover(message='%s meter2inverter: %s '%(role,s), title='KAS must stop', priority=1)

		exit_thread(kas_con)



######################
# keep alive client
# if cannot bind after n try, exit
# tries FOREVER to connect to KAS
# send PING. if error , exit
# receive with timeout. after n timeout, exit
# blink led on data received
# if global set (by main) exit
######################

def keep_alive_client(x):
	global stop_thread
	global stop_main

	s = 'KAC: start on port: %d' %(port_ip+2)
	logging.info(s)
	print(s)

	error = 0
	max_error = 2 # 

	kac = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

	# bind 
	for i in range(5):
		try:
			s = 'KAC: trying %d bind on port %d' %(i, port_ip+2)
			logging.info(s)
			print(s)

			kac.bind(("", port_ip+2))

			s = 'KAC: bound %s' %str(kac)
			logging.info(s)
			print(s)

			break

		except Exception as e:
			if i < 4:
				#kac.close()
				#kac.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
				sleep(15)
			else:

				s = 'KAC: %d cannot bind  %d %s' %(i, port_ip+2, str(e))
				logging.error(s)
				print(s)
				pushover.send_pushover(message='%s meter2inverter: %s '%(role,s), title='KAC cannot bind', priority=1)

				oled_text('KAC: BIND ERROR',2) # line
				oled_text('EXIT',1) # line

				# exit from own will. also tell main to sys.exit
				exit_thread(kac)


	# socket bound, try to connect FOREVER (timeout)
	while True:
		try:
			kac.connect((remote_ip, port_ip+1))
			s = 'KAC: connected'
			print(s)
			logging.info(s)
			break

		except KeyboardInterrupt:
			print('CTRL C while trying to connect to Keep Alive Server')
			clean()
			sys.exit(0)

		except Exception as e:
			print('KAC: retry to connect', str(e))
			sleep(5)


	# send ping, wait for pong with timeout 
	kac.settimeout(5)

	while True:
		# send random int to ka server
		r = randrange(256)
		try:
			kac.sendall(r.to_bytes(2, byteorder='big')) # need bytes, not int
			# or struct.pack(">I", 1) '\x00\x00\x00\x01'
			# struct.pack("<H", 1)'\x01\x00'
			# struct.pack("B", 1)'\x01'
			error = 0
		except Exception as e:
			# cannot send PING [Errno 32] Broken pipe if remote killedee

			s = 'KAC: send ER'
			oled_text(s,2) # line
			oled_text('EXIT',1) # line

			s = s + str(e) 
			logging.error(s)
			print(s)
			pushover.send_pushover(message='%s meter2inverter: %s '%(role,s), title='KAC send ERROR', priority=1)

			exit_thread(kac)

		# expect pong within timeout
		try:
			b = kac.recv(4)
			# get b'' if remote killed, interpreted as 0, so generates a missmatch.

			b1 = int.from_bytes(b,"big")
			# should get pong with same content

			if (b1 != r):
				# got something within timeout, but content not correct
				s = 'KAC: missmatch ping %d pong %d' %(r, b1) 
				logging.error(s)
				print(s)
			else:
				blink_led(led_alarm ,0.01) # pulse led at every valid pong received 


		except Exception as e:
			s = 'KAC: timeout on pong %s' % str(e)
			logging.error(s)
			print(s)
			# did no get response

			error = error + 1
			if error > max_error:
				# consider remote dead
				s = 'KAC: pong missing error %d %s' %(error, str(e)) 
				logging.error(s)
				print(s)

				oled_text('no PONG',2) # line
				oled_text('EXIT',1) # line
				pushover.send_pushover(message='%s meter2inverter: %s '%(role,s), title='KAC no PONG', priority=1)


				exit_thread(kac)

			else:
				pass


		# check if main is asking the thread to exit
		if stop_thread == True:
			# global set by main
			print("KAC: stop_thread", stop_thread)

			s = 'KAC: must STOP'
			oled_text(s,1) # line
			oled_text('EXIT',2) # line

			logging.error(s)
			print(s)
			pushover.send_pushover(message='%s meter2inverter: %s '%(role,s), title='KAC must stop', priority=1)

			exit_thread(kac)


		else:
			sleep(ping_frequency)



####################
# serial to socket
# recv with timeout
# stats, led
# write to log once
# send to socket
# if exception, exit
####################


def serial_to_socket(x):
	global stop_thread
	global stop_main

	do_once_serial = True
	nb_serial_recv = 0 # number of read returning, can be more than bytes, less than MODBUS frames

	print('start serial to socket thread')


	dict = rs.get_settings()
	print(dict)

	if role == 'gx' or role == "em":

		#rs.reset_input_buffer() # Flush input buffer, discarding all its contents.

		# wait until getting 0x01 0x03
		s = '%s waiting for 0x01 0x03' %role
		logging.info(s)
		print(s)

		got_0x01 = False

		rs.reset_input_buffer() # Flush input buffer, discarding all its contents.

		while True:
			b = rs.read(1) # return bytes
			if len(b) == 1:
				if int(b[0])== 0x01:
					got_0x01 = True
					print(b)

				if int(b[0]) == 0x03 and got_0x01:
					break

			if len(b) > 1:
				b0 = int(b[0])
				b1 = int(b[1])
				if b0 == 0x01 and b1 == 0x03:
					break

		s = '%s got 0x01 0x03 %s' %(role,str(b))
		logging.info(s)
		print(s)
		pushover.send_pushover(message='%s meter2inverter: %s '%(role,s), title=s, priority=0)

	buffer = [0x01,0x03] # list of int
	in_frame=False

	while True:

		try:

			b = rs.read(nb_serial) # return bytes

			# serial read with timeout larger than 1.5x char
			# Read size bytes from the serial port. If a timeout is set it may return fewer characters than requested

			if b != b'':

				#print(b)

				##### log/print once
				if do_once_serial:
					oled_text('serial data',1)

					# write ONCE in log we got some traffic
					s = 'ONCE: received data from serial: %s' %str(b)
					logging.info(s)
					print(s)
					do_once_serial = False

				# stats, led
				blink_led(led_traffic,pulse_led) # pulse traffic led at every receive, ie we are alive

				# number of read. each read can contains multiple bytes
				s = 'Serial> %d' %nb_serial_recv

				# print when nb_serial is a multiple of modulo (eg 50)
				# will print 1st time (nb_serial = 0)
				if (nb_serial_recv % modulo) == 0:
					print(s)
					oled_text(s,2)   # stats on line 2
				else:
					pass

				if nb_serial_recv == 65000:
					nb_serial_recv = 0
				else:
					nb_serial_recv = nb_serial_recv + 1


				# got something 
				in_frame=True

				for i in range(len(b)): # bytes is a sequence. for return int,
					buffer.append(int(b[i]))

				#print(buffer)


			else: # ''
				# got b'' timeout on serial constructor

				if in_frame:

					# send buffer to socket
					try:
						# need bytes type, not a list
						#buff = b''.join(buffer)

						buff = bytearray(buffer) # convert list of int into bytes type
						#print("sending buff: ", buff, type(buff))

						con.sendall(buff)

						in_frame=False
						buffer = []

					except Exception as e:
						s = 'SOCK SEND ERR'
						oled_text(s,2)
						oled_text('EXIT',1)

						# BrokenPipeError: [Errno 32] Broken pipe
						s = s + str(e)
						print(s)
						logging.error(s)
						pushover.send_pushover(message='%s meter2inverter: %s '%(role,s), title='SOCK send error', priority=1)

						exit_thread(con)

				else: # no serial data, and not in frame
					pass


		except Exception as e:
			s = 'serial2socket ERR'
			oled_text(s,2)
			oled_text('EXIT',1)

			s = s + str(e)
			print(s)
			logging.error(s)
			pushover.send_pushover(message='%s meter2inverter: %s '%(role,s), title='serial2socket error', priority=1)

			exit_thread(con)

	if stop_thread:
		# global set by main
		print("serial2socket: stop_thread", stop_thread)

		s = 'serial2socket: must STOP'
		oled_text(s,1) # line
		oled_text('EXIT',2) # line

		logging.error(s)
		print(s)
		pushover.send_pushover(message='%s meter2inverter: %s '%(role,s), title=s, priority=1)

		exit_thread(con)

	# while True



####################
# socket to serial 
# recv with timeout
# stats, led
# write to log once
# send to socket
# if exception, exit
####################

def socket_to_serial(x):
	global stop_thread
	global stop_main
	do_once_wifi = True
	nb_socket_recv = 0

	print('start socket to serial thread')

	while True:

		try:
			b = con.recv(nb_socket)
			if b != b'':

				# log once
				if do_once_wifi:
					# write ONCE in log we got some traffic
					s = 'ONCE: received data from wifi %s' %str(b)
					logging.info(s)
					print(s)
					pushover.send_pushover(message='%s meter2inverter: %s '%(role,s), title=s, priority=0)
					do_once_wifi = False

				# stats, led
				blink_led(led_traffic,pulse_led)

				# number of read. each read can contains multiple bytes
				s = 'Socket> %d' %nb_socket_recv

				if nb_socket_recv % modulo == 0:
					oled_text(s,3)    # stats on line 3
					print(s)
				else:
					pass

				if nb_socket_recv == 65000:
					nb_socket_recv = 0
				else:
					nb_socket_recv = nb_socket_recv + 1

				# send
				rs.write(b) # assume no exception


		except Exception as e:

			if str(e) == 'timed out':
				pass # timeout reading socket
			else:
				s = 'socket2serial ERR'
				oled_text(s,2)
				oled_text('EXIT',1)

				s = s + str(e)
				print(s)
				logging.error(s)
				pushover.send_pushover(message='%s meter2inverter: %s '%(role,s), title='socket2serial error', priority=1)

				exit_thread(con)

	if stop_thread:
		# global set by main
		print("socket2serial: stop_thread", stop_thread)

		s = 'socket2serial: must STOP'
		oled_text(s,1) # line
		oled_text('EXIT',2) # line

		logging.error(s)
		print(s)
		pushover.send_pushover(message='%s meter2inverter: %s '%(role,s), title=s, priority=1)

		exit_thread(con)

	# while True



################################
################################
# MAIN
################################
################################


print('blink yellow led at start')
for i in range(5):
	blink_led(led_traffic,0.1)


# _  for python3
stop_thread = False # set to true in main to ask thread to exit 
stop_main = False # set to true in thread in case of problems, to ask main to exit

if arg["keep_alive"]:
	print("start keep alive threads")
	id1= _thread.start_new_thread(keep_alive_server, ('pabou',))
	id2= _thread.start_new_thread(keep_alive_client, ('pabou',))
else:
	print("do NOT start keep alive threads")

###############
# socket, serial
##############

# timeout in sec for serial
print('serial timeout sec:' , timeout_serial)

# USB serial host to energy meter's USB RS485

if role == 'em': # use PI USB port to connect to EM  
	port="/dev/ttyUSB0" # exists if RS485 USB connected to pi
if role == 'gx': # use FTDI adapter to connect to GX. connects on PI UART
	port="/dev/ttyAMA0" # TX gpio 8, RX gpio 10

try:
	"""
	timeout = None: wait forever / until requested number of bytes are received
	timeout = 0: non-blocking mode, return immediately in any case, returning zero or more, up to the requested number of bytes
	timeout = x: set timeout to x seconds (float allowed) returns immediately when the requested number of bytes are available, otherwise wait until the timeout expires and return all bytes that were received until then.
	"""
	rs = serial.Serial(
            port=port,
            baudrate=9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout_serial,
            inter_byte_timeout = None,
            bytesize=8)

	s = 'serial ' + str(rs)

	print(s)
	logging.info(s)

except Exception as e:
	s = role + 'no serial'

	oled_text(s,2)
	oled_text('EXIT',1)

	s = s + str(e)
	print(s)
	logging.error(s)
	pushover.send_pushover(message='%s meter2inverter: %s '%(role,s), title='no serial', priority=1)

	exit_main()


net = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# socket not released immediatly ? bind error for CLIENT
for i in range(5):
	try:
		net.bind(("", port_ip))
		s= "socket bound " + str(net)
		print(s)
		logging.info(s)
		break

	except Exception as e:

		if i == 4:
			s = '%d cannot bind to port %d %s' %(i, port_ip, str(e))
			print(s)
			logging.error(str(datetime.datetime.now())+ s)

			oled_text('BIND ERROR',2)
			oled_text('EXIT',1)
			pushover.send_pushover(message='%s meter2inverter: %s '%(role,s), title='BIND error', priority=1)

			exit_main()

		else:
			print('bind error on main socket, wait')
			#net.close()
			#net.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
			sleep(15)


# GX initiate connection

if role == 'em':   # server

	# wait FOREVR for a connection

	s = 'EM accept'
	print(s)
	logging.info(s)

	oled_text(s, 1) # overwrite line 1 to leave 2 lines on smaller oled
	oled_text('', 2) # erase 

	net.listen()

	# blocking only ? MOVE TO ASYNCH to insert check reboot
	con, addr = net.accept() # use con to send/rec

	s = 'EM connected' # tuple

	oled_text(s, 1) # overwrite line 1 to leave 2 lines on smaller oled
	oled_text('', 2) # erase 

	s = s + str(addr)
	print(s)
	logging.info(s)

	GPIO.output(led_alarm,0) 

	# 1st data home made signaling
	data = con.recv(256) # returns bytes. max amount data received at once
	print('1st data: ',data, type(data))

if role == 'gx': 

	# try FOREVER to connect to EM

	s = 'GX connecting ..'
	oled_text(s,1)
	oled_text('', 2) # erase 

	while True: # keep trying to connect to EM
		try:
			net.connect((remote_ip,port_ip)) # 
			s = 'GX connected'

			print(s)
			logging.info(s)

			oled_text(s,1)
			oled_text('', 2) # erase 

			GPIO.output(led_alarm,0) 
			break

		except Exception as e:

			check_reboot()
			sleep(5) # try again connecting to EM

	# while true

	"""
	c = net.send(data)
	if c != len(data):
		print('could not send all data ', c, len(data))
	"""

	# send some signaling data, will not be relayed to serial on other side
	c = net.sendall(version.encode('utf-8')) # or b'version 1.0'

	con = net


###############################
# RELAY
###############################


# blink led to signal start of relay, ie connection established
for i in range(3):
	blink_led(led_traffic,0.5)

#con.setblocking(0)
print('socket recv timeout sec: ', timeout_socket)
con.settimeout(timeout_socket) # socket timeout on recv in sec

s = 'start meter2inverter relay'
print(s)
logging.info(s)

start = time() # float nb sec

do_once_serial = True
do_once_wifi = True

nb_serial_recv=0 # count number of frame received
nb_socket_recv=0

stop_main=False
stop_thread=False

id3= _thread.start_new_thread(serial_to_socket, ('pabou',))
id4= _thread.start_new_thread(socket_to_serial, ('pabou',))


# after starting threads, main only check for reboot

while True:

	try:
		# reboot button GPIO 21 and GND. pull up
		check_reboot()

		if stop_main == True:
			s = 'stop main True, main must exit'
			logging.info(s)
			print(s)
			pushover.send_pushover(message='%s meter2inverter: %s '%(role,s), title=s, priority=1)
			oled_text(s,2)
			oled_text('EXIT',1)

			exit_main()

		sleep(5)

	except Exception as e:

		stop_thread=True
		print("exception in main while true. asking all thread to exit", stop_thread)

		s = "exception main %s" %str(e)
		logging.error(s)
		print(s)
		pushover.send_pushover(message='%s meter2inverter: %s '%(role,s), title=s, priority=1)
		oled_text(s,2)
		oled_text('EXIT',1)

		exit_main()

	else:
		pass

	finally:
		pass  # executed at each loop

print('end')





"""
# ftdi from GX to raspberry USB /dev/ttyUSB0
# 2 bytes per registers, 1st byte MSB 

start serial to USB relay
# x00 X01  read 1 register, = 1 word 
# W dmd  LSB  W dmd peak MSB
47274.1 serial to EM: 0 b b'\x01\x03\x00\x0b\x00\x01\xf5\xc8'

0x02 quantity of requested bytes (nb register *2)
38.5 EM to serial: b'\x01\x03\x02\x00x\xb8f'

# try  slave 02
62.4 serial to EM: 0 b b'\x02\x03\x00\x0b\x00\x01\xf5\xfb'

# 7 registers from 0x5000 serial number 
253.5 serial to EM: 50 0 b'\x01\x03P\x00\x00\x07\x15\x08'
# response 14 bytes 0 '02'  0 '04' 0 '00' 0 '04' 0 '03' 0 
171.8 EM to serial: b'\x01\x03\x0e\x002\x004\x000\x004\x003\x00'

# ????
26.0 EM to serial: b'4\x00V\xa7\xa1'

# slave 2
37.0 serial to EM: 0 b b'\x02\x03\x00\x0b\x00\x01\xf5\xfb'

#0x0303 1 register revision code
246.8 serial to EM: 3 3 b'\x01\x03\x03\x03\x00\x01tN'
#rev 4
42.4 EM to serial: b'\x01\x03\x02\x00\x04\xb9\x87'

# slave 2
99.7 serial to EM: 0 b b'\x02\x03\x00\x0b\x00\x01\xf5\xfb'

# 0x1103 measurement mode selection 
258.3 serial to EM: 11 3 b'\x01\x03\x11\x03\x00\x01q6'
# tariff 1
38.7 EM to serial: b'\x01\x03\x02\x00\x01y\x84'

36.3 serial to EM: 0 b b'\x02\x03\x00\x0b\x00\x01\xf5\xfb'

# code 0x06 write single holding register tariff 1
258.0 serial to EM: 11 3 b'\x01\x06\x11\x03\x00\x01\xbd6'

255.4 serial to EM: 0 b b'\x02\x03\x00\x0b\x00\x01\xf5\xfb'

256.9 serial to EM: 11 3 b'\x01\x06\x11\x03\x00\x01\xbd6'

# response tariff 1 (echo request)
40.2 EM to serial: b'\x01\x06\x11\x03\x00\x01\xbd6'

37.7 serial to EM: 0 b b'\x02\x03\x00\x0b\x00\x01\xf5\xfb'

# 0x0004 2 reg Watt and VA each int32
255.1 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
# missing nb of bytes in response ? 
238.4 EM to serial: b'\x01\x03\x00\x00\x80\x03\x04\x00\x00\x00\x00\xfa3'

# resent ?
15.2 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
27.8 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'

# ??
14.5 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa'

# Watt VA
239.6 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
# ?? resp = 00
38.8 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'

39.3 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
37.1 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'
217.4 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
37.2 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'
219.8 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
38.9 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'
211.2 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
34.2 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'

# 4 reg starting 0 V A W VA
36.0 serial to EM: 0 0 b'\x01\x03\x00\x00\x00\x04D\t'

# 8 bytes
36.8 EM to serial: b'\x01\x03\x08\t"\x00\x00\x00\x00\x00\x00W\xbf'

# W VA
144.6 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
# 4 bytes 0
36.5 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'

203.3 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
37.1 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'
224.5 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
37.2 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'
205.3 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
38.3 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'
38.4 serial to EM: 0 10 b'\x01\x03\x00\x10\x00\x03\x04\x0e'
38.7 EM to serial: b'\x01\x03\x06\x00\x01\x00\x00\x00\x00\x1c\xb5'
150.0 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
40.9 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'
195.6 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
38.6 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'
214.5 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
39.4 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'
215.4 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
40.4 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'
36.8 serial to EM: 0 20 b'\x01\x03\x00 \x00\x03\x04\x01'
45.9 EM to serial: b'\x01\x03\x06\x00\x00\x00\x00\x00\x00!u'
124.3 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
42.3 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'
208.1 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
37.0 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'
224.1 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
36.8 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'
203.5 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
36.1 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'
224.1 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
38.9 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'
206.8 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
38.6 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'
216.2 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
40.1 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'
217.2 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
40.0 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'
37.2 serial to EM: 0 0 b'\x01\x03\x00\x00\x00\x04D\t'
39.4 EM to serial: b'\x01\x03\x08\t!\x00\x00\x00\x00\x00\x00d\xbf'
125.9 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
40.8 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'
217.4 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
41.3 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'
190.5 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
41.0 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'
210.7 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
35.3 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'
34.4 serial to EM: 0 10 b'\x01\x03\x00\x10\x00\x03\x04\x0e'
37.4 EM to serial: b'\x01\x03\x06\x00\x01\x00\x00\x00\x00\x1c\xb5'
140.6 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
36.8 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'
224.8 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
34.9 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'
224.3 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
37.0 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'
206.5 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
38.2 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'
59.9 serial to EM: 0 20 b'\x01\x03\x00 \x00\x03\x04\x01'
37.9 EM to serial: b'\x01\x03\x06\x00\x00\x00\x00\x00\x00!u'
104.5 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
39.5 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'
216.5 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
39.8 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'
218.6 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
41.3 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'
212.8 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
39.2 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'
211.7 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
39.5 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'
212.5 serial to EM: 0 4 b'\x01\x03\x00\x04\x00\x02\x85\xca'
36.2 EM to serial: b'\x01\x03\x04\x00\x00\x00\x00\xfa3'


"""
