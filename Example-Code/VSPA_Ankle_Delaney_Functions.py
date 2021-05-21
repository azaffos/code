# Variable Stiffness Ankle Functions
# Delaney Miller
# Summer 2017

# Description: Helper functions to run homing routine and move ankle to desired position

# Dependencies: time, numpy, csv, wiringpi, LS7366R.py

# Import functions
import time
import numpy
import csv
import wiringpi as wp
from LS7366R import LS7366R

# Label pins
pwm_pin = 18
dir_pin = 25
enable_pin = 24
disable_pin = 23

# Initialize motor encoder
CSX = 1			# chip select channel (0 or 1)
CLK = 1000000	# SPI clock speed (0.5 MHz)
BTMD = 4		# bytemode resolution of counter (1-4)
encoder = LS7366R(CSX, CLK, BTMD)
scale = 12578.0 # encoder conversion scale (counts to mm)

# Stole these functions from Max's code (max_functions_pi.py)

# Initialize ankle angle encoder
SPIchannel = 0 #SPI Channel (CE0)
SPIspeed = 500000 #Clock Speed in Hz
wp.wiringPiSPISetupMode(SPIchannel, SPIspeed,1)
my_data = numpy.zeros((50000, 2))

#Clear error flag?
data1 = 0b01000000
foo1 = chr(data1)
data2 = 0b00000001
foo2 = chr(data2)
foobar = ''.join([foo1, foo2])
sendData = foobar
recvData = wp.wiringPiSPIDataRW(SPIchannel, sendData)

def lpfilter(x,ypast):
    b = [4.165461390757824e-04,0.001249638417227,0.001249638417227,4.165461390757824e-04]
    a = [1,-2.686157396548143,2.419655110966473,-0.730165345305723]
    y = -(a[1]*ypast[0] + a[2]*ypast[1] + a[3]*ypast[2]) + b[0]*x[0] + b[1]*x[1] + b[2]*x[2] + b[3]*x[3];
    return(y)

def lpfilterDelaneyDelaney(x,ypast):
	b = [2.413590490419615e-04, 4.827180980839230e-04, 2.413590490419615e-04]
	a = [1, -1.955578240315036, 0.956543676511203]
	y = -(a[1]*ypast[0] + a[2]*ypast[1]) + b[0]*x[0] + b[1]*x[1] + b[2]*x[2];
	return(y)

def lpfilterDMillz(x,ypast):
	b = [0.046131802093313, 0.092263604186626, 0.046131802093313]
	a = [1, -1.307285028849324, 0.491812237222575]
	y = -(a[1]*ypast[0] + a[2]*ypast[1]) + b[0]*x[0] + b[1]*x[1] + b[2]*x[2];
	return(y)
    
def representsInt(s):	#Just checks to see if it can be converted from string to int
    try: 
        int(s)
        return True
    except ValueError:
        return False

def SingleAngle():	#reports back the current ankle angle
	foo1 = chr(0b11111111)
	foo2 = chr(0b11111111)
	sendData = ''.join([foo1, foo2])
	recvData = wp.wiringPiSPIDataRW(SPIchannel, sendData)
	#Need to do it a second time to get the READ
	foo1 = chr(0b11111111)
	foo2 = chr(0b11111111)
	sendData = ''.join([foo1, foo2])
	recvData = wp.wiringPiSPIDataRW(SPIchannel, sendData)
	datas = recvData[1]
	datachars = list(datas)
	data1 = bin(ord(datachars[0]))
	data2 = bin(ord(datachars[1]))
	rawbin1 = data1[2:]
	rawbin2 = data2[2:]
	bitstring1 = -len(rawbin1) % 8 * '0' + rawbin1
	bitstring1 = '00' + bitstring1[2:]
	bitstring2 = -len(rawbin2) % 8 * '0' + rawbin2
	byte1 = int(bitstring1,2)
	byte2 = int(bitstring2,2)
	value1 = byte1*256 + byte2
	encoder_offset = 258.75
	angle = value1*360.0/16384.0 - encoder_offset
	return angle

def CollectAngleData(sample_time, sampling_frequency, stiffness):
	my_data = numpy.zeros((50000, 2))
	experiment_duration = sample_time
	i = 0
	time_elapsed = 0
	foo1 = chr(0b11111111)
	foo2 = chr(0b11111111)
	sendData = ''.join([foo1, foo2])
	recvData = wp.wiringPiSPIDataRW(SPIchannel, sendData)
	foo1 = chr(0b11111111)
	foo2 = chr(0b11111111)
	print('Beginning Data Collection')
	start_time = time.time()
	while time_elapsed < experiment_duration:
		i = i+1
		t0 = time.time()
		sendData = ''.join([foo1, foo2])
		recvData = wp.wiringPiSPIDataRW(SPIchannel, sendData)
		#recvData now holds a list [NumOfBytes, recvDataStr] e.g. [2, '\x9A\xCD']

		datas = recvData[1]
		datachars = list(datas)
		data1 = bin(ord(datachars[0]))
		data2 = bin(ord(datachars[1]))
		rawbin1 = data1[2:]
		rawbin2 = data2[2:]
		bitstring1 = -len(rawbin1) % 8 * '0' + rawbin1
		bitstring1 = '00' + bitstring1[2:]
		bitstring2 = -len(rawbin2) % 8 * '0' + rawbin2
		byte1 = int(bitstring1,2)
		byte2 = int(bitstring2,2)
		value1 = byte1*256 + byte2
		encoder_offset = 258.75
		angle = value1*360.0/16384.0 - encoder_offset

		t1 = time.time()
		# print(sendData, angle, t1-t0, 'hi')
		time.sleep(1.0/sampling_frequency-0.001)

		time_elapsed = time.time()-start_time
		my_data[i,0] = time_elapsed
		my_data[i,1] = angle


	my_data = my_data[:i]
	print("Done recording. Data stored in Max_Pi_data.csv")

	column_names = ['time','angle']
	with open("Max_Pi_data.csv", "w") as output:
	    writer = csv.writer(output,delimiter=',')
	    writer.writerow(['stiffness:',stiffness])
	    writer.writerow(column_names)
	    writer = csv.writer(output, lineterminator='\n',quotechar='|')
	    writer.writerows(my_data[1:i,0:8])




# Here is my own function to move the slider


def sliderPosition(x_des):
	dist = x_des - encoder.readCounter()

	# Label pins
	pwm_pin = 18
	dir_pin = 25
	enable_pin = 24
	disable_pin = 23

	# Define gains for PID control
	K_p = 0.08 * 0.45
	K_i = 0.0195
	K_d = 0.0001

	e_D = 0
	iTerm = 0
	last_pwm = 0
	# x2 = [0.0]*5; xpast2 = [0.0]*5
	
	# track data
	my_data = numpy.zeros((50000, 8))
	i = 0
	start_time = time.time()
	last_time = start_time
	dist_last = dist

	# current monitoring
	R = 0.791  # in Ohms
	K_v = 1470 # in rpm/V
	V_supply = 12  # in V
	P_v = 0
	# b = [0.0]*5; bpast = [0.0]*5

	# Initialize variables
	current_time = time.time()
	current_position = encoder.readCounter()

	print "Press CTRL + C to return to desired position menu."

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
			if -100 < last_pwm < 100:
				iTerm = iTerm + (K_i * dist * dt)
			# # if iTerm > 100 or iTerm < -100:
			# # 	iTerm = last_iTerm

			# e_D = (dist-dist_last)/(dt)
			# x.pop()
			# x.insert(0,e_D)
			# e_D_filtered = lpfilterDelaneyDelaney(x,xpast)
			# xpast.pop()
			# xpast.insert(0,e_D_filtered)
			# dTerm = e_D_filtered * K_d

			e_D2 = (dist-dist_last)/(dt)
			# x2.pop()
			# x2.insert(0,e_D2)
			# e_D_filtered2 = lpfilterDMillz(x2,xpast2)
			# xpast2.pop()
			# xpast2.insert(0,e_D_filtered2)
			dTerm = e_D2 * K_d
			
			pwm_feedback = int(pTerm + iTerm + dTerm)

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
				my_data[i,5] = int(iTerm)
				my_data[i,6] = int(dTerm)
				my_data[i,7] = I_inst
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
			break

	wp.pwmWrite(pwm_pin, 0)
	my_data = my_data[:i]
	print("Done recording. Data stored in my_data.csv")

	with open("my_data.csv", "w") as output:
		writer = csv.writer(output,delimiter=',')
		writer = csv.writer(output, lineterminator='\n',quotechar='|')
		writer.writerows(my_data[1:i,0:9])