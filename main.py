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

i2cbus = SMBus(1)
i2c_addr = 0x77
t_fine = 0.0
Data = namedtuple('Data', ['temperature', 'humidity','pressure'])
CompData = namedtuple('CompData',['digt', 'digh', 'digp'])

def fetch_compensation_params():
    """
    Fetches all the raw calibrated data
    Table 16 P.24 need to read the data from the registers.  can read upto
    dig_P9.  Which is 24 bits so the Temp and pressure compensation data
    :rtype: a list of raw calibration data
    """
    raw_comp_params = []

    for i in range (0x88, 0x88+24):
        raw_comp_params.append(i2cbus.read_byte_data(i2c_addr,i))
    # need to read the first bit of the Humidity data
    raw_comp_params.append(i2cbus.read_byte_data(i2c_addr,0xA1))
    # read the rest of the Humidity data E1 -E6 so 7 bits
    for i in range(0xE1, 0xE1+7):
        raw_comp_params.append(i2cbus.read_byte_data(i2c_addr,i))
    return raw_comp_params

def reorder_compensation_params(raw_comp_params):
    """
    the compenastion params are stored in reverse order.
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
    return CompData(digT, digH, digP)

# now that the order of the bits has been made usable.  I need to convert negative 2's complment
# into negative integers.
def from_twos_complement(bit_string, num_bits=16):
    unsigned = int(bit_string, 2)
    sign_mask = 1 << (num_bits -1)
    bits_mask = sign_mask - 1
    return (unsigned & bits_mask) - (unsigned & sign_mask)

def bit_length(num):
    """
    find the length of an integer that is represented as a binary string.  Need to take into account the leading ob
    chars
    :param num: an integer
    :return: integer representing the bit length of the integer
    """
    if num.bit_length() > 8:
        return 16
    elif num.bit_length() > 2:
        return 8

def fetch_raw_sensor_data():
    data = []
    for i in range(0xF7, 0xFC+8):
        data.append(i2cbus.read_byte_data(i2c_addr,i))
    return data

def process_raw_sensor_data():
    raw_data = fetch_raw_sensor_data()
    pres_raw = (raw_data[0] << 12 | (raw_data[1] << 4) | raw_data[2] >> 4)
    temp_raw = (raw_data[3] << 12 | (raw_data[4] << 4) | raw_data[5] >> 4)
    hum_raw = (raw_data[6] << 8) | raw_data[7]
    return Data(temp_raw, hum_raw, pres_raw)

def read_all():
    data = process_raw_sensor_data()
    return Data(
        read_temperature(data),
        read_humidity(data),
        read_pressure(data)
    )
def read_temperature(data):
    comp_temp = compensate_temp()
def compensate_temp(temp,comp_temp):
    global t_fine
    var1 = (temp /16384) - (comp_temp[0]/1024) * comp_temp[1]
    var2 = (temp /131072) - (comp_temp[0]/8192) * (temp/131072 - comp_temp[0]/8192) * comp_temp[2]
    t_fine = var1 + var2
    temperature = t_fine / 5120
    return temperature

def compensated_pressure(pres,comp_press):
    var1 = (t_fine/2.0) - 64000.0
    var2 = var1 * var1 * comp_press[5] /32768.0
    var2 = var2 + var1 * (comp_press[4] * 2)
    var2 = (var2 / 4.0) + (comp_press[3] * 65536.0)
    var1 = (1.0 +var1 / 32768.0) * comp_press[0]
    if var1 == 0.0:
        return 0
    p = 1048576.0 - pres
    p = (p - (var2 / 4096.0)) * (6250.0 / var1)
    var1 = comp_press * p * p / 2147483648.0
    p = p + (var1 + var2 + comp_press[6]) / 16
    return p

def compensated_humidity(hum, comp_hum):
    var_h = t_fine - 76800.0
    var_h = (hum - (comp_hum[3] * 64.0 + comp_hum[4] / 16384.0 * var_h)) * (
        comp_hum[1] / 65536.0 * (1.0 + comp_hum[5] / 67108864.0 * var_h * (
        1.0 + comp_hum[2] / 67108864.0 * var_h)))
    var_h = var_h * (1.0 - comp_hum[0]) * var_h / 524288.0
    if var_h > 100.0:
        var_h = 100.0
    elif var_h < 0.0:
        var_h = 0.0
    return var_h

if __name__== '__main__':
    processed = reorder_compensation_params(fetch_compensation_params())
    for i in (1,2):
        if processed['digT'][i] & 0b1000000000000000:
            processed['digT'][i] = from_twos_complement(processed['digT'][i], bit_length(processed['digT'][i]))
            print(processed['digT'][i])

