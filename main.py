# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import time

from smbus import SMBus
from collections import namedtuple

# Define  registers for BME280
HUM_LSB = 0xFE
HUM_MSB = 0xFD
TEMP_xlsb = 0xFC
TEMP_LSB = 0xFB
PRESS_XLSB = 0xF9
PRESS_LSB = 0xF8
PRESS_MSB = 0xF7
CONFIG = 0xF5
CTRL_MEAS = 0xF4
STATUS = 0xF3
CTRL_HUM = 0xF2
RESET = 0xE0
ID = 0xD0

class Bme280():

    t_fine = 0.0
    def __init__(self,setup=0, i2c_addr=0x77):
        self.setup = setup
        self.i2c_addr = i2c_addr
        self.i2cbus = SMBus(1)
        self.compensatedData = self._getCallibrationData()
        self.sensorData = {}
        print(self.compensatedData)

    def fetch_compensation_params(self):
        """
        Fetches all the raw calibrated data
        Table 16 P.24 need to read the data from the registers.  can read upto
        dig_P9.  Which is 24 bits so the Temp and pressure compensation data
        :rtype: a list of raw calibration data
        """
        raw_comp_params = []

        for i in range (0x88, 0x88+24):
            raw_comp_params.append(self.i2cbus.read_byte_data(self.i2c_addr,i))
        # need to read the first bit of the Humidity data
        raw_comp_params.append(self.i2cbus.read_byte_data(self.i2c_addr,0xA1))
        # read the rest of the Humidity data E1 -E6 so 7 bits
        for i in range(0xE1, 0xE1+7):
            raw_comp_params.append(self.i2cbus.read_byte_data(self.i2c_addr,i))
        return raw_comp_params

    def reorder_compensation_params(self, raw_comp_params):
        """
        the compensation params are stored in reverse order across multiple registers.
        Two registers need to be combined to make a single number. To represent
        the Temperature, humidty and pressure.
        do this by shifting 8 bits 8 bits to the left.  Combine the
        second number by OR'ing the next register onto the shifted number.
        The order of the registers and their content are described in the data sheet
        Table 16 page 24
        according to Table 16 page 24
        digT[0] is an unsigned WORD 16bit
        digT[1,2] is a signed WORD and is stored in TWO's complement 16bit
        digP[0] is an unsigned WORD and is stored in TWO's complement 16bit
        dig[1-9] are signed word and stored in 2's complement 16bit
        digH[0] is an unsigned WORD 8bit
        digH[1] is a signed WORD 16bit
        digH[2] is an unsigned binary 8bit
        digH[3,4] are signed binary 16bit digits
        :param raw_comp_params: contains a list of 24 bits.
        :return:
        """
        digT = []
        digH = []
        digP = []
        digT.append((raw_comp_params[1] << 8) | raw_comp_params[0])
        digT.append((raw_comp_params[3] << 8) | raw_comp_params[2])
        digT.append((raw_comp_params[5] << 8) | raw_comp_params[4])
        digP.append((raw_comp_params[7] << 8) | raw_comp_params[6])
        digP.append((raw_comp_params[9] << 8) | raw_comp_params[8])
        digP.append((raw_comp_params[11] << 8) | raw_comp_params[10])
        digP.append((raw_comp_params[13] << 8) | raw_comp_params[12])
        digP.append((raw_comp_params[15] << 8) | raw_comp_params[14])
        digP.append((raw_comp_params[17] << 8) | raw_comp_params[16])
        digP.append((raw_comp_params[19] << 8) | raw_comp_params[18])
        digP.append((raw_comp_params[21] << 8) | raw_comp_params[20])
        digP.append((raw_comp_params[23] << 8) | raw_comp_params[22])
        digH.append(raw_comp_params[24])
        digH.append((raw_comp_params[26] << 8) | raw_comp_params[25])
        digH.append(raw_comp_params[27])
        # # the last two number are created from using 1 register E5 so we need to create the
        # correct mask and only shift bits four places.
        digH.append((raw_comp_params[28] << 4) | (0x0F & raw_comp_params[29]))
        digH.append((raw_comp_params[30] << 4) | (raw_comp_params[29] >> 4 & 0x0F))
        digH.append(raw_comp_params[31])
        return {'temperature': digT, 'humidity': digH, 'pressure':digP}

    # now that the order of the bits has been made usable.  I need to convert negative 2's complment
    # into negative integers.

    def comp_data(self, processed):
        """
        sorting out all the compensated data so that it is all ordered and can be used by the actual temp,hum, pressure
        data.
        Checking to see if the data from the wire is a negative number.  If it is converting it from a twos' complement
        to a normal negative number.
        :return:
        """
        for i in (1,2):
            if processed['temperature'][i] & 0x8000: # checking to see if the left most digit is 1 use bit wise &
                processed['temperature'][i] = (~processed['temperature'][i] ) + 1
        for i in (1,8):
            if processed['pressure'][i] & 0x8000:
                processed['pressure'][i] = (~processed['pressure'][i]) + 1
        for i in (0,5):
                if processed['humidity'][i] & 0x8000:
                    processed['humidity'][i] = (~processed['humidity'][i]) + 1
        return processed

    def _getCallibrationData(self):
        """
        Method that just calls the steps to get and process the compensated data.  Then stores the data in a class
        attribute
        :return: a dictionary of comparison data
        """
        comp_data = self.reorder_compensation_params(self.fetch_compensation_params())
        print(comp_data)
        return self.comp_data(comp_data)

    def fetch_raw_sensor_data(self):
        data = []
        for i in range(0xF7, 0xFC+8):
            data.append(self.i2cbus.read_byte_data(self.i2c_addr,i))
        return data

    def process_raw_sensor_data(self,raw_data):
        pres_raw = (raw_data[0] << 12 | (raw_data[1] << 4) | raw_data[2] >> 4)
        temp_raw = (raw_data[3] << 12 | (raw_data[4] << 4) | raw_data[5] >> 4)
        hum_raw = (raw_data[6] << 8) | raw_data[7]
        return {'temperature': temp_raw, 'humidity': hum_raw, 'pressure': pres_raw}

    def compensate_temp(self, temp, comp_temp):
        global t_fine
        var1 = (temp /16384.0 - comp_temp[0]/1024.0) * comp_temp[1]
        var2 = (temp /131072.0 - comp_temp[0]/8192.0) * (temp/131072.0 - comp_temp[0]/8192.0) * comp_temp[2]
        t_fine = var1 + var2
        temperature = t_fine / 5120.0
        return temperature

    def compensated_pressure(self, pres, comp_press):
        var1 = (t_fine/2.0) - 64000.0
        var2 = (((var1 / 4.0) * (var1 / 4.0)) / 2048) * comp_press[5]
        var2 += ((var1 * comp_press[4]) * 2)
        var2 = (var2 / 4.0) + (comp_press[3] * 65536.0)
        var1 = (((comp_press[2] * (((var1 /4.0) * (var1 / 4.0))/ 8192))/ 8) +((comp_press[1] * var1) / 2.0)) / 262144
        var1 = ((32768 + var1) * comp_press[0]) / 32768
        if var1 == 0.0:
            return 0
        p = ((1048576.0 - pres) - (var2 / 4096)) * 3125
        if pres < 0x80000000:
            pres = (pres * 2.0) / var1
        else:
            pres = (pres / var1) * 2

        var1 = (comp_press[8] * (((pres / 8.0) * (pres  / 8.0)) / 8192.0)) / 4096
        var2 = ((pres / 4.0) * comp_press[7]) / 8192.0
        p += ((var1 + var2 + comp_press[6]) / 16.0)
        return p / 100

    def compensated_humidity(self, hum, comp_hum):
        var_h = t_fine - 76800.0
        if var_h ==0:
            return 0
        var_h = (hum - (comp_hum[3] * 64.0 + comp_hum[4] / 16384.0 * var_h)) * (
            comp_hum[1] / 65536.0 * (1.0 + comp_hum[5] / 67108864.0 * var_h * (
            1.0 + comp_hum[2] / 67108864.0 * var_h)))
        var_h *= (1.0 - comp_hum[0] * var_h / 524288.0)
        if var_h > 100.0:
            var_h = 100.0
        elif var_h < 0.0:
            var_h = 0.0
        return var_h

    def read_all(self):
        data = self.fetch_raw_sensor_data()
        data = self.process_raw_sensor_data(data)
        self.sensorData = {'temperature': self.read_temperature(data['temperature']),
                           'humidity': self.read_humidity(data['humidity']),
                           'pressure': self.read_pressure(data['pressure'])
                }

    def read_pressure(self,sensor_data):
        comp_press = self.compensated_pressure(sensor_data, self.compensatedData['pressure'])
        return comp_press
    def read_humidity(self,sensor_data):
        comp_hum = self.compensated_humidity(sensor_data, self.compensatedData['humidity'])
        return comp_hum

    def read_temperature(self,sensor_data):
        comp_temp = self.compensate_temp(sensor_data, self.compensatedData['temperature'])
        return comp_temp

    def weather_setup(self):
        """
        send the setup for the recommended weather station setting
        sensor mode forced mode
        overssampling pressure x1 temp x1 humidity x 1
        Memory map snippet from table 18: page 27
        CTRL_Meas | bit 7 |bit 6| bit 5| bit 4| bit 3 | bit 2 | bit 1 | bit 0  |
        __________|     osrs_t[2:0]    |       osrs_p[2:0]    | mode[1:0]
        Status    | bit 7 |bit 6| bit 5| bit 4| bit 3 | bit 2 | bit 1 | bit 0  |
        __________|     t_sb[2:0]      |       filter[2:0]    |xxxxxxx|spi3w[0]|
        CTRL_hum  | bit 7 |bit 6| bit 5| bit 4| bit 3 | bit 2 | bit 1 | bit 0  |
                  |xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx|   osrs_h[2:0]          |
        IIR Filter off
        :return: 0
        """
        osrs_h = 0x1
        osrs_p = 0x1
        osrs_t = 0x1
        mode = 0x1
        filter = 0x0
        # different modes need sleep to initially set the mode
        sleep = 0x00
        t_sb = 0x5
        spi3w = 0x0

        ctrl_meas = (osrs_t << 5)| (osrs_p << 2) | mode
        ctrl_hum = osrs_h
        status = (t_sb << 5) | (filter << 2) | spi3w
        self.i2cbus.write_byte_data(self.i2c_addr, CTRL_MEAS, sleep)
        self.i2cbus.write_byte_data(self.i2c_addr,CTRL_HUM, ctrl_hum)
        self.i2cbus.write_byte_data(self.i2c_addr,CTRL_MEAS, ctrl_meas)
        self.i2cbus.write_byte_data(self.i2c_addr, STATUS, status)



    def initialise_bme280(self):
        """
        Read the compensation data.
        Set the BME280 to the Weather station scenario.
        :return:
        """
        self.weather_setup()

if __name__== '__main__':
    sens = Bme280()
    sens.read_all()
    print(sens.sensorData)