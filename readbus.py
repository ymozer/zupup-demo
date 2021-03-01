from multiprocessing import Process, Pipe, Queue, current_process
from subprocess import Popen, PIPE, call
import testspi
import time

tempsensorID="spi0.0"
if __name__ == '__main__':
    p= current_process()
    print('Starting:', p.name, p.pid)

    while True:
        t=time.time()
        time.sleep(.5)
        print(tempsensorID)
        elapsed = "%.2f" % (time.time() - t)
        mys = testspi.testspi(tempsensorID)
        print(mys.readTempC())
