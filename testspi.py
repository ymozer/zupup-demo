from subprocess import Popen, PIPE, call
import os

class testspi:
    numSensor = 0
    def __init__(self, tempSensorId):
        self.tempSensorId = tempSensorId
        self.sensorNum = testspi.numSensor
        # Raspbian build in January 2015 (kernel 3.18.8 and higher) has changed the device tree.
        oldOneWireDir = "/sys/bus/spi/devices/w1_bus_master1/"
        newTempSPIDir = "/sys/bus/spi/devices/"
        if os.path.exists(oldOneWireDir):
            self.oneWireDir = oldOneWireDir
        else:
            self.TempSPIDir = newTempSPIDir
        print("Constructing SPI sensor %s"%(tempSensorId))

    def readTempC(self):
        #pipe = Popen(["cat","/sys/bus/w1/devices/" + tempSensorId + "/w1_slave"], stdout=PIPE)
        pipe = Popen(["cat", self.TempSPIDir + self.tempSensorId + "/"], stdout=PIPE)
        result = pipe.communicate()[0]


        return result
