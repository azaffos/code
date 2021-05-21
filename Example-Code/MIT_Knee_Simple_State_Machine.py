#!/usr/bin/python

import smbus, time, math, random, threading
from Adafruit_ADS1x15 import ADS1x15
import MPU6050_ER as mpu6050
import BLNKM as LED
from tml import TML
import RPi.GPIO as GPIO

board = TML('/dev/ttyAMA0')

# ============================================================================
# Data acquisition code
# ============================================================================

# Configure i2c baudrate
#os.system("sudo modprobe -r i2c_bcm2708 && sudo modprobe i2c_bcm2708 baudrate=1000000")
#subprocess.call(shlex.split('sudo modprobe -r i2c_bcm2708 && sudo modprobe i2c_bcm2708 baudrate=1400000'))

# Initialise the ADC using the default mode (IC = ADS1115, default address)
adc = ADS1x15()

from DataLogger import dataLogger
dl = dataLogger()
data = []

mpu6050.initialize()
board.startUp()
time.sleep(0.01)
LED.initialize()
GPIO.setmode(GPIO.BOARD)
#Setup Comm Pin
GPIO.setup(12,GPIO.OUT)
GPIO.output(12, GPIO.HIGH)
#Setup Brake Pinout
GPIO.setup(16,GPIO.OUT)
GPIO.output(16,GPIO.HIGH)

board.setVar('KDP',0)
board.setVar('KPP',0)

print 'starting...'
t0 = time.time()
tb = time.time()

brake = 1
brakeflag = 1

i = 0.0;
board.read()

while time.time() - t0 < 5:
    i = i + 1
    t1 = time.time() - t0
    GPIO.output(12, GPIO.LOW) # 1.8 ms delay between pin voltage change 
    time.sleep(0.0015)
    v1 = adc.readADCSingleEnded(0) # Significant testing on 3/13/13 indicates that channels 0/1 are switched in HW or library software
    time.sleep(0.0015)
    AccYZ = mpu6050.getAccYZ()
    Ang = math.atan2(AccYZ[0],AccYZ[1])
    DrvVals = board.getVarInterrupt()
    v0 = adc.readADCSingleEnded(1)
    print DrvVals
    if DrvVals[0] > (2**32)/2: # Account for rollover APOS position variable
        DrvVals[0] = DrvVals[0] - (2**32) 
    if DrvVals[1] > (2**16)/2: # Account for rollover of IQ current variable 
        DrvVals[1] =  DrvVals[1] - (2**16)
    if DrvVals[2] > (2**16)/2: # Account for rollover of UQREF voltage variable
        DrvVals[2] = DrvVals[2] - (2**16)
    
    DrvVals[0] = (DrvVals[0]*(360.0/(4.0*500.0)))/145.1 # Converting to degrees, scaling factor Kpf, from Technosoft Control Manual, Section 3.7, then dividing by gear ratio
    DrvVals[1] = DrvVals[1]*((2.0*20.0)/65520.0) # Converting to amps, from email from Gabriel to Elliott on 1/29/13
    DrvVals[2] = DrvVals[2]*(24.0/65534.0) # Converting to volts, scaling factor Kuf, from Technsoft Manual, Section 3.7
    print DrvVals
    #print DrvVals[2]
    v0 = (v0*(360.0/5.0))-207.3 # Converting to degrees, using datasheet
    t2 = time.time() - t0
    GPIO.output(12, GPIO.HIGH)
    GyroX = mpu6050.getGyroX()
    if brake == 1:
        if brakeflag == 1:
            print 'Brake ON...'
            GPIO.output(16, GPIO.LOW)
            LED.fadetoRGB(0,255,0)
            brakeflag = 0
        if time.time() - tb > 0.65:
            brake = 0
            tb = time.time()
            KDPflag = 1
            nobrakeflag = 1
    if brake != 1:
        if nobrakeflag == 1:
            print 'Brake OFF...'
            GPIO.output(16, GPIO.HIGH)
            LED.fadetoRGB(50,50,255)
            nobrakeflag = 0
        if (time.time() - tb > .550) & KDPflag == 1:
            board.setVar('KDP',0)
            KDPflag = 0
        if (time.time() - tb) > .85:
            brake = 1
            board.setVar('KDP',0)
            tb = time.time()
            brakeflag = 1
    if DrvVals != None: 
        data = [i] + [t1] + [brake+0.0] + [v0] + [v1]  + [AccYZ[0]] + [AccYZ[1]] + [GyroX[0]] + [DrvVals[0]] + [DrvVals[1]] + [DrvVals[2]] + [t2]
        # data = [i] + [t1] + [brake+0.0] + [v0] + [v1]  + [DrvVals[0]+0.0] + [DrvVals[1]+0.0] + [DrvVals[2]+0.0] + [t2]
    else:
        data = [i] + [t1] + [brake+0.0] +  [0.0] + [0.0] + [0.0] + [0.0] + [0.0] + [0.0] + [0.0] + [0.0] + [t2] 
        print 'No data was stored'
    dl.appendData(data)
print i
print "Knee Angle: %.2f " % (v0)
print "Load: %.2f " % (v1)
print "AccY: %.2f " % (AccYZ[0])
print "AccZ: %.2f " % (AccYZ[1])
print "GyroX: %.2f " % (GyroX[0])
print "Motor Angle: %.2f " % (DrvVals[0])
print "Motor Current: %.2f " % (DrvVals[1])
print "Motor Voltage: %.2f " % (DrvVals[2])
LED.fadetoRGB(0,0,0)
dl.writeOut("VoltageCurrentTest1.txt")
