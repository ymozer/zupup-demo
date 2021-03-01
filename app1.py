from flask import Flask
import max6675
from multiprocessing import Process, Value

app = Flask(__name__)

# set the pin for communicate with MAX6675
cs = 24
sck = 23
so = 21

celsius = 1

# max6675.set_pin(CS, SCK, SO, unit)   [unit : 0 - raw, 1 - Celsius, 2 - Fahrenheit]
max6675.set_pin(cs, sck, so, 1)


@app.route('/')
def index():
    snt = f'Sıcaklık: {celsius}'
    return snt


def temp():
    try:
        while 1:
            global celsius
	    celsius = max6675.read_temp(cs)
#            print(celsius)
            max6675.time.sleep(2)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    thread.start_new_thread(temp, ())
    app.run(debug=True, host='0.0.0.0', use_reloader=False)
