import time
import math
import mpu6050
import BLNKM as LED


# Sensor initialization
mpu = mpu6050.MPU6050()
mpu.dmpInitialize()
mpu.setDMPEnabled(True)
LED.initialize()
    
# get expected DMP packet size for later comparison
packetSize = mpu.dmpGetFIFOPacketSize()
avgx = []
avgy = []
avgz = []
prev_sum = int(0)
sleep_flag = 0

# Program loop
while True:
    # Get INT_STATUS byte
    mpuIntStatus = mpu.getIntStatus()
  
    if mpuIntStatus >= 2: # check for DMP data ready interrupt (this should happen frequently)
        fifoCount = mpu.getFIFOCount() # get current FIFO count
        
        # check for overflow (this should never happen unless our code is too inefficient)
        if fifoCount == 1024:
            mpu.resetFIFO() # reset so we can continue cleanly

	time.sleep(0.01)
        # wait for correct available data length, should be a VERY short wait
        fifoCount = mpu.getFIFOCount()
        while fifoCount < packetSize:
            fifoCount = mpu.getFIFOCount()
        
        result = mpu.getFIFOBytes(packetSize)
        q = mpu.dmpGetQuaternion(result)
        g = mpu.dmpGetGravity(q) # Gravity vector
        
        avgx.append(g['x'])
        avgy.append(g['y'])
        avgz.append(g['z'])

        if len(avgx) > 10:
        	avgx.pop(0)
        	avgy.pop(0)
        	avgz.pop(0)
        x = int(math.fabs((sum(avgx)/len(avgx)) * 255))
        y = int(math.fabs((sum(avgy)/len(avgy)) * 255))
        z = int(math.fabs((sum(avgz)/len(avgz)) * 255))

        
        if math.fabs((x + y + z) - prev_sum) < 5:
            if sleep_flag == 0:
                t_idle = time.time()
                sleep_flag = 1
            if sleep_flag == 1:
                if time.time() - t_idle > 10:
                    LED.fadetoHSB(170,200,60)
                else:
                    LED.fadetoRGB(x,y,z)
        else:
            sleep_flag = 0
            LED.fadetoRGB(x,y,z)

        prev_sum = x + y + z



        print x, y, z

        time.sleep(0.1)

        fifoCount -= packetSize 
        
