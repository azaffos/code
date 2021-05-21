#!/usr/bin/python

# PID Test Code
# Delaney Miller
# Summer 2017

# Import functions
import time
import numpy
import csv
import wiringpi as wp
from LS7366R import LS7366R
from delaney_functions import *
# from PID import PID
# import matplotlib.pyplot as plt
# from scipy.interpolate import spline


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
max_pos = 80;  # maximum position in mm (need to test this)

# If unable to read in the motor encoder position, quit the program
try:
	current_position = encoder.readCounter() / scale
except:
	print "Something is wrong with the motor encoder. Going to quit in 5 seconds."
	encoder.close()
	time.sleep(5)
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


# PID control

position_desired = int(raw_input('Enter a desired position in mm (0 to 50):'))

x_des = position_desired * scale
x_act = encoder.readCounter()
dist = int(x_des - x_act)

K_p = 0.032
# K_i = 0.0195
# K_d = 0.0001

e_D = 0
iTerm = 0
last_pwm = 0
x2 = [0.0]*5; xpast2 = [0.0]*5
my_data = numpy.zeros((50000, 8))
i = 0
start_time = time.time()
last_time = start_time
dist_last = dist

R = 0.791  # in Ohms
K_v = 1470 # in rpm/V
V_supply = 12  # in V
b = [0.0]*5; bpast = [0.0]*5

# Initialize variables
current_time = time.time()
current_position = encoder.readCounter()
P_v = 0   # power dissipation



while True:
	try:


		last_position = current_position
		last_time = current_time

		current_time = time.time()
		dt = current_time - last_time

		current_position = encoder.readCounter()
		v_rot = (current_position - last_position) / dt  # in counts/s
		v_rot = v_rot * 60 / 4096  # in rpm
		back_emf = v_rot / K_v  # calculate back emf
		# b.pop()
		# b.insert(0,back_emf)
		# back_emf_filtered = lpfilter(b,bpast)
		# bpast.pop()
		# bpast.insert(0,back_emf_filtered)

		pTerm = K_p * dist

		# integral windup
		# if -100 < last_pwm < 100:
		# 	iTerm = iTerm + (K_i * dist * dt)

		# e_D = (dist-dist_last)/(dt)
		# x.pop()
		# x.insert(0,e_D)
		# e_D_filtered = lpfilterDelaneyDelaney(x,xpast)
		# xpast.pop()
		# xpast.insert(0,e_D_filtered)
		# dTerm = e_D_filtered * K_d

		# e_D2 = (dist-dist_last)/(dt)
		# x2.pop()
		# x2.insert(0,e_D2)
		# e_D_filtered2 = lpfilterDMillz(x2,xpast2)
		# xpast2.pop()
		# xpast2.insert(0,e_D_filtered2)
		# dTerm = e_D2 * K_d
		
		pwm_feedback = int(pTerm)
		# pwm_feedback = int(pTerm + iTerm + dTerm)

		# limit pwm feedback to between 7% and 100% duty cycle
		if pwm_feedback > 100:
			pwm_feedback = 100
		elif pwm_feedback < -100:
			pwm_feedback = -100
		elif 2 < pwm_feedback <= 7:
			pwm_feedback = 7
		elif -7 <= pwm_feedback < -2:
			pwm_feedback = -7
		elif -2 <= pwm_feedback <= 2: 
			pwm_feedback = 0

		# calculate instantaneous current
		V_in = V_supply * (pwm_feedback / 100)
		I_inst = (V_in - back_emf) / R

		pwm = abs(pwm_feedback)

		# store some data
		t_elapsed = time.time() - start_time
		if t_elapsed < 1.5:
			my_data[i,0] = t_elapsed
			my_data[i,1] = pwm_feedback
			my_data[i,2] = dist
			my_data[i,3] = encoder.readCounter()
			my_data[i,4] = int(pTerm)
			# my_data[i,5] = int(iTerm)
			# my_data[i,6] = int(dTerm)
			i = i+1

		# figure out direction
		
		if pwm_feedback < 0:
			wp.digitalWrite(dir_pin, 0)
		elif pwm_feedback > 0:
			wp.digitalWrite(dir_pin, 1)

		
		wp.pwmWrite(pwm_pin, pwm)

		# update error term
		x_act = encoder.readCounter()
		dist_last = dist
		dist = int(x_des - x_act)
		last_time = current_time
		last_pwm = pwm_feedback
		last_position = current_position

	except KeyboardInterrupt:
		wp.pwmWrite(pwm_pin, 0)
		my_data = my_data[:i]
		print("Done recording. Data stored in my_data.csv")

		column_names = ['time','pwm','error','position','K_p']

		with open("my_data.csv", "w") as output:
			writer = csv.writer(output,delimiter=',')
			writer.writerow(column_names)
			writer = csv.writer(output, lineterminator='\n',quotechar='|')
			writer.writerows(my_data[1:i,0:5])
		break

wp.pwmWrite(pwm_pin, 0)