import Plot
import Sensors.picam as picam
import Sensors.picam360 as picam360
from time import sleep
import sys
import Adafruit_GPIO.SPI as SPI  # Import Adafruit GPIO_SPI Module
import Adafruit_MCP3008  # Import Adafruit_MCP3008
import os
#from picamera import PiCamera


def main():
    #camera = PiCamera()
    interval = float(input('At what interval (in seconds) would you like to collect data?: '))
    runtime = int(input('What would you like the total runtime to be (in seconds)?: '))
    filename = input("What name would you like to give the file/ what dir (ex: test.csv)?: ")
    pin_num = 6  # Must be hard coded

    filename = 'data/' + filename
    #data = collect_data(interval, runtime, pin_num, filename, camera)
    data = collect_data(interval, runtime, pin_num, filename)

    if data != -1:

        save_data(data, filename)  # saves the data to a file


#def collect_data(interval, runtime, pin_num, filename, camera):
def collect_data(interval, runtime, pin_num, filename):

    '''
       params
       interval in seconds can be an int or float
       runtime how long you want to collect data (if runtime = -1 it will continue until you C-c out)

       data will only be collected if interval < runtime, interval > 0, and runtime != -1

       ret
       data is interval at idx 0 then collected data is at data[1:]
       if data is not collected retruns -1
       '''
    # We can either use Software SPI or Hardware SPI. For software SPI we will
    # use regular GPIO pins. Hardware SPI uses the SPI pins on the Raspberry PI
    # Set the following variable to either HW or SW for Hardware SPI and Software
    # SPI respectivly.

    # PIN IS 6 AS A PARAMETER
    SPI_TYPE = 'HW'
    # dly = .5  # Delay of 1000ms (1 second) original
    dly = .5 * interval

    # Software SPI Configuration
    CLK = 18  # Set the Serial Clock pin
    MISO = 23  # Set the Master Input/Slave Output pin
    MOSI = 24  # Set the Master Output/Slave Input pin
    CS = 25  # Set the Slave Select

    # Hardware SPI Configuration
    HW_SPI_PORT = 0  # Set the SPI Port. Raspi has two.
    HW_SPI_DEV = 0  # Set the SPI Device

    # Instantiate the mcp class from Adafruit_MCP3008 module and set it to 'mcp'.
    if (SPI_TYPE == 'HW'):
        # Use this for Hardware SPI
        mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(HW_SPI_PORT, HW_SPI_DEV))
    elif (SPI_TYPE == 'SW'):
        # Use this for Software SPI
        mcp = Adafruit_MCP3008.MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)

    analogPort = pin_num
    print('Reading MCP3008 values on pin: %d' % analogPort)

    data = [interval]
    try:
        if runtime == -1:
            while True:
                # Read the value from the MCP3008 on the pin we specified in analogPort
                val = mcp.read_adc(analogPort)
                # print out the value
                print(val)
                # Sleep for dly
                sleep(dly)
        else:
            t = 0
            i = 0
            while t < runtime:
                # Read the value from the MCP3008 on the pin we specified in analogPort
                val = mcp.read_adc(analogPort)
                # print out the value
                print(val)
                data.append(val)
                os.system("fswebcam " + filename + str(i) + ".jpg")
                # Sleep for dly
                sleep(dly)
                t += interval
                i += 1
    except KeyboardInterrupt:
        sys.exit()
    return data


def save_data(data, filename):
    '''
    params
    data should be a list
    name is a string of what you want the file to be named
    path is a string where you want the csv file to be saved

    ex: save_data_to_csv([3,2,1], text.csv, ../data/) will save a file named test.csv
    to the dir the is 1 dir back and then into the data dir and it will contain 3,2,1
    '''
    f = open(filename, 'w')
    data = str(data)[1:-1]
    f.write(data)
    f.close()
    Plot.main()


if __name__ == '__main__':
    main()
