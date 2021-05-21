# Variable Stiffness Ankle Code
# Delaney Miller
# August 2017

# Description: Runs program to change stiffness of ankle, check current position, etc.
# Based on Ankle_Run.py, written by Max Shepherd.

# Dependencies: time, numpy, csv, wiringpi, LS7366R, delaney_functions

# Import functions
import time
import numpy
import csv
import wiringpi as wp
from LS7366R import LS7366R
from delaney_functions import *


# Set up wiringpi pins to be GPIO
wp.wiringPiSetupGpio()

# Assign motor driver pins and set modes
pwm_pin = 18
dir_pin = 25
enable_pin = 24
disable_pin = 23
wp.pinMode(enable_pin, 1)
wp.pinMode(disable_pin, 1)
wp.pinMode(dir_pin, 1)
wp.pinMode(pwm_pin, 2)

# Initialize motor encoder
CSX = 1			# chip select channel (0 or 1)
CLK = 1000000	# SPI clock speed (0.5 MHz)
BTMD = 4		# bytemode resolution of counter (1-4)
encoder = LS7366R(CSX, CLK, BTMD)
scale = 12578.0 # encoder conversion scale (counts to mm)

# If unable to read in the motor encoder position, quit the program
try:
	current_position = encoder.readCounter() / scale
except:
	print "Something is wrong with the motor encoder. Going to quit."
	encoder.close()
	time.sleep(1)
	quit()

# Reset motor driver and configure PWM pin
wp.digitalWrite(enable_pin, 0)
wp.digitalWrite(disable_pin, 1)
wp.pwmWrite(pwm_pin, 0)
wp.digitalWrite(dir_pin, 0)
time.sleep(1)
wp.digitalWrite(enable_pin, 1)
wp.digitalWrite(disable_pin, 0)
wp.pwmSetMode(0)
wp.pwmSetRange(100)

# 1. Run encoder homing routine
homing = str(raw_input('Okay to proceed with homing routine to set slider position? (y/n) '))

if homing == 'n':
	print "Unable to determine position. Going to quit."
	encoder.close()
	time.sleep(1)
	quit()

if homing == 'y':
	try:
		print "Starting homing routine. Press Ctrl + C to terminate homing program."
		time.sleep(1)
		# homing_data = numpy.zeros((50000, 2))
		# i = 0
		# current_time = time.time()
		last_position = encoder.readCounter()
		sample_rate = 0.05
		keep_going = True
		wp.pwmWrite(pwm_pin, 20)
		time.sleep(0.1)
			
		while keep_going:
			last_position = encoder.readCounter()
			time.sleep(sample_rate)
			current_position = encoder.readCounter()
			error = current_position - last_position
			# my_data[i,0] = time.time() - current_time
			# my_data[i,1] = error
			if -100 <= error <= 100:
				keep_going = False
			# i = i + 1
		wp.pwmWrite(pwm_pin, 0)

		encoder.clearCounter()
		current_position = int(round(encoder.readCounter() / scale))
		print "Homing successful. Slider position: %s mm" % current_position

		# homing_data = homing_data[:i]
		# print("Done recording. Data stored in homing_data.csv")

		# with open("homing_data.csv", "w") as output:
			# writer = csv.writer(output,delimiter=',')
			# writer = csv.writer(output, lineterminator='\n',quotechar='|')
			# writer.writerows(my_data[1:i,0:8])

	except KeyboardInterrupt:
		wp.pwmWrite(pwm_pin, 0)
		encoder.close()
		quit()

# 2. Run ankle program
while True:
	to_do = str(raw_input('What do you want to do? (stiffness, position, ankle angle, quit)'))

	if to_do == 'quit':
		print "Okay, quitting!"
		encoder.close()
		break

	if to_do == 'position':
		current_position = int(round(encoder.readCounter() / scale))
		print "The current slider position is %s mm" % current_position

	if to_do == 'stiffness':
		print "Stiffness: type 'back' at any time to return to the main menu or enter a position."
		while True:
			user_input = raw_input('Desired position in mm (0 to 50): ')
			current_position = encoder.readCounter() / scale
			
			# Return to main menu if user types 'back'
			if user_input == 'back':
				break

			# Check if an integer was printed
			if representsInt(user_input):
				user_input = eval(user_input)
				if 0 <= user_input <= 50:
					
					# HAVE TO CHANGE THIS LINE WHEN I START TO ACTAULLY USE THE ANKLE ANGLE ENCODER
					current_angle = 0    
					if -2 < current_angle < 2:
						desired_position = user_input * scale

						# Here is where we run the actual control loop
						sliderPosition(desired_position)
						current_position = int(round(encoder.readCounter()/scale))
						print "Current position is %s mm" % current_position

					else:
						print "Ankle is flexed. Waiting for swing."
						made_it = 0
						for x in xrange(100):
							time.sleep(0.02)
							current_angle = SingleAngle()
							if -2 <= current_angle < 2:
				   				desired_position = user_input * scale
								sliderPosition(desired_position)
								current_position = int(round(encoder.readCounter()/scale))
								print "Current position is %s mm" % current_position
								made_it = 1
								break
							if made_it != 1:
								print 'Sorry, the ankle stayed flexed :('
				else:
					print "Out of range. Please enter a value between 0 and 50."
			else:
				print "Please enter an integer value."
				break

	if to_do == 'ankle angle':
		current_angle = SingleAngle()
		print 'angle = ', current_angle
		print 'stiffness =', current_position