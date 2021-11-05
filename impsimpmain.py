import os, sys
from flexseapython.pyFlexsea import *
from flexseapython.fxUtil import *
def main():
	print('')
	scriptPath = os.path.dirname(os.path.abspath(__file__))
	fpath = scriptPath + '/flexseapython/com.txt'
	ports, baudRate = loadPortsFromFile(fpath)
	print('Loaded ports: ' + str(ports))
	print('Using baud rate: ' + str(baudRate))

	# Control gain constants

	kp = 100
	ki = 32
	K = 325
	B = 1


	devId = fxOpen('/dev/ttyACM0', 230400, 6)
	resolution = 500
	fxStartStreaming(devId, resolution, True)

	result = True

	data = fxReadDevice(devId)
	initialAngle = data.encoderAngle


	fxSendMotorCommand(devId, FxImpedance, initialAngle)

        # Set gains

	fxSetGains(devId, kp, ki, 0, K, B)



        # Record start time of experiment

	while True:
		data = fxReadDevice(devId)
		measuredPos = data.encoderAngle
#		fxSendMotorCommand(devId, FxImpedance, initialAngle)
#		preamble = "Holding position: {}...".format([data.encoderAngle])
		preamble = "Motor Current: {}...".format([data.motorCurrent])
		print(preamble) 
main()

