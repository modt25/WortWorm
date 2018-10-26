import micropython
import machine
import time
import math
from ustruct import unpack as unp
import onewire, ds18x20
import urequests
import sys
#from ubidots.apiclient import ApiClient

# Constants
ADC_PIN = 0
DS18X20_PIN = 12 #D6 = GPIO12

I2C_SCL_PIN = 2 #D4
I2C_SDA_PIN = 0 #D3
I2C_FREQ = 100000 # GY521 can go up to 400kHz per datasheet
GY521_ADDR = 0x68 #from datasheet and i2c.scan() method while testing
ACCEL_RANGE = 0 #0 = 14 bit = 16384
GYRO_RANGE = 0

## Register addresses for accelerometer, gyro, and temp sensor on GY521
# Per GY521 datasheet
RA_ACCEL_XOUT_H   = 0x3B
RA_ACCEL_XOUT_L   = 0x3C
RA_ACCEL_YOUT_H   = 0x3D
RA_ACCEL_YOUT_L   = 0x3E
RA_ACCEL_ZOUT_H   = 0x3F
RA_ACCEL_ZOUT_L   = 0x40
RA_TEMP_OUT_H     = 0x41
RA_TEMP_OUT_L     = 0x42
RA_GYRO_XOUT_H    = 0x43
RA_GYRO_XOUT_L    = 0x44
RA_GYRO_YOUT_H    = 0x45
RA_GYRO_YOUT_L    = 0x46
RA_GYRO_ZOUT_H    = 0x47
RA_GYRO_ZOUT_L    = 0x48

ADC_MULTIPLIER = 5.5/1024 #voltage divider / resolution of ADC ~ 191.0 - exact by meassuring precise resistances/voltages
ADC_RESOLUTION = 12 # 12bit resolution == 750ms update rate
LOW_BATT_VALUE = 3.3
#CFGFILE = '/config.json'

UBIDOTS_API_KEY = '43f1740bb15a78d54ab39cefdf4e19a1ba115ec9'
UBIDOTS_API_TOKEN = '8WCHleYc8dOJMSihJoSblmIWS1rIhu'
UBI_TEMP_LABEL = 'temp'
UBI_TEMP_ID = '58cf63ca7625427363b3e7da'
UBI_TILT_LABEL = 'tilt'
UBI_TILT_ID = '58d5bdd57625426f0d1fa372'
UBI_VOLT_LABEL = 'battery'
UBI_VOLT_ID = '58da722c7625427df34e7ed9'
UBI_SPECGRAV_LABEL = ''
UBI_SPECGRAV_ID = ''
UBI_DEVICE_LABEL = 'wortworm'

SLEEP_TIME = 10000 # 1 hour between measurements

#ZERO_RESET_CODE = 0x4321
#ONE_RESET_CODE = 0x1234
DBL_RST_TIMEOUT = 5000

#define OWinterval (800 / (1 << (12 - RESOLUTION)))
#define TKIDSIZE 35
#define MEDIANROUNDS 7
#define ACCINTERVAL 200
#define MEDIANAVRG 3

#define CBP_ENDPOINT (PSTR("/api/hydrometer/v1/data"))

## sleep management
#define RTCSLEEPADDR 5
#define MAXSLEEPTIME 3600 //TODO
#define EMERGENCYSLEEP (my_sleeptime*3 < MAXSLEEPTIME ? MAXSLEEPTIME : my_sleeptime*3)

SSID = 'ziemodt'

def do_connect():
    import network
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('connecting to network...')
        wlan.connect(SSID, 'exc1t0ns')
        while not wlan.isconnected():
            pass
    print('network config:', wlan.ifconfig())

class GY521:
    def __init__(self, scl_pin, sda_pin, freq=100000, accel_range=0, gyro_range=0):
        self.i2c = machine.I2C(scl=machine.Pin(scl_pin), sda=machine.Pin(sda_pin), freq=freq)
        self.accel_range = accel_range
        self.gyro_range = gyro_range

        #confirm device on bus
        roms = self.i2c.scan()
        if not roms: #empty list check
            print('ERROR: No I2C device(s) found.')
            pass

        for rom in roms:
            if (rom == GY521_ADDR):
                print('Found GY521 sensor')
                self.i2c.writeto_mem(GY521_ADDR, 0x3B, '\x80') #reset
                self.wake()

    def wake(self):
        '''
        Wakes the device.
        '''
        self.i2c.writeto_mem(GY521_ADDR, 0x3B, '\x01')
        return 'awake'


    def sleep(self):
        '''
        Put the device to sleep
        '''
        self.i2c.writeto_mem(GY521_ADDR, 0x3B, '\x40')
        return 'asleep'

    def getAccelValues(self):
        scale = [2**14, 2**13, 2**12, 2**11]
        self.axyz_raw = self.i2c.readfrom_mem(GY521_ADDR, 0x3B, 6)

        self.accel = {'x': unp('>h', self.axyz_raw[0:2])[0]/scale[self.accel_range],
                      'y':unp('>h', self.axyz_raw[2:4])[0]/scale[self.accel_range],
                      'z':unp('>h', self.axyz_raw[4:6])[0]/scale[self.accel_range]
                     }
        return self.accel

    def getGyroValues(self):
        self.gx = self.i2c.readfrom_mem(GY521_ADDR, RA_GYRO_XOUT_H, 1) << 8 | self.i2c.readfrom_mem(GY521_ADDR, RA_GYRO_XOUT_L,1)
        self.gy = self.i2c.readfrom_mem(GY521_ADDR, RA_GYRO_YOUT_H, 1) << 8 | self.i2c.readfrom_mem(GY521_ADDR, RA_GYRO_YOUT_L,1)
        self.gz = self.i2c.readfrom_mem(GY521_ADDR, RA_GYRO_ZOUT_H, 1) << 8 | self.i2c.readfrom_mem(GY521_ADDR, RA_GYRO_ZOUT_L,1)

    def getTemperature(self):
        self.temp = self.i2c.readfrom_mem(GY521_ADDR, RA_TEMP_OUT_H, 2)

    def calculateTilt(self):  #averages for final value?
        # Assume we have accel x,y,z values
        pitch = math.atan2(self.accel['y'], math.sqrt(self.accel['x']**2 + self.accel['z']**2))
        roll  = math.atan2(self.accel['x'], math.sqrt(self.accel['y']**2 + self.accel['z']**2))
        return math.sqrt(pitch**2 + roll**2)

class DoubleResetDetector():
    def __init__(self, timeout = 1000):
        self.timeout = timeout

    def create_timer(self):
        drd_timer = machine.Timer(1)
        drd_timer.init(period = self.timeout, mode=machine.Timer.ONE_SHOT, callback = self.timer_expired)

    def timer_expired(self, drd_timer):
        print('DRD Timeout!')
        self.clearRecentResetFlag()

    def detectDoubleReset(self):
        detectDR = self.detectRecentResetFlag()
        if detectDR:
            print('Double Reset Detected')
            self.clearRecentResetFlag()
        else: #first reset
            self.setRecentResetFlag()
            #start timer
        return detectDR

    def detectRecentResetFlag(self):
        doubleResetDetectorFlag = machine.RTC().memory()
        doubleResetDetected = (doubleResetDetectorFlag == b'First Reset')
        return doubleResetDetected

    def setRecentResetFlag(self):
        print('First Reset')
        machine.RTC().memory(b'First Reset')
        self.create_timer()

    def clearRecentResetFlag(self):
        print('Clearing DRD')
        machine.RTC().memory(b'')


class Ubidots():
    """
    Ubidots object representing one device on the Ubidots cloud data service for storing and plotting data
    """
    def __init__(self, api_token, device_label):
        self.api_token = api_token
        self.api_URL = 'http://things.ubidots.com/api/v1.6/'
        self.device_label = device_label

    def send(self, var_label, value):
        url = self.api_URL + 'devices/%s/?token=%s' % (self.device_label, self.api_token)
        #print(url)
        payload = {var_label:value}
        header = {'Accept': 'application/json',"Content-Type": "application/json"}
        #print(payload)
        r = urequests.post(url, json = payload, headers=header)
        #print(r.content)
        #var = self.api.get_variable(var_id)
        #var.save_value({'value': value})

    def send_all(self, dict_values):
        """
        dict_values is dictionary of {'var_label':value, 'var2_label':value, ...} to be sent
        """
        url = self.api_URL + 'devices/%s/?token=%s' % (self.device_label, self.api_token)
        header = {'Accept': 'application/json',"Content-Type": "application/json"}
        payload = dict_values
        r = urequests.post(url, json = payload, headers = header)

    def get(self, var_id):
        pass


class WortWorm():
    def __init__(self, GY521, DS_sensor, Battery_sensor, ubidots_device_label, SleepTimer = machine.RTC()):
        # GY521 gyro/accelerometer sensor connected via I2C on Pins 2 (SCL) and 0 (SDA)
        self.accel = GY521

        # The DS18x20 temperature sensor object is on GPIO12
        self.ds = DS_sensor

        # ADC object for reading battery voltage
        self.batt = Battery_sensor

        #ubidots
        self.ubi = Ubidots(api_token=UBIDOTS_API_TOKEN, device_label = ubidots_device_label)
        self.var_ids = {'temp': UBI_TEMP_ID, 'tilt': UBI_TILT_ID, 'battery': UBI_VOLT_ID}

        # Deep sleep timer
        self.sleep_timer = SleepTimer

    def __calibrateSG__(self):
        pass

    # for taking temperatures with the DS18x20
    def getTemperature(self):
        addr = self.ds.scan()
        self.ds.convert_temp()
        time.sleep_ms(750) #Wait a minimum of 750ms before readout
        return self.ds.read_temp(addr[0]) #assuming only one device attached

    def getBatteryVoltage(self):
        self.batt.read() #drop first read
        volt = self.batt.read() #ESP Input 0-1V, reads 0-1024
        return volt * ADC_MULTIPLIER

    def getTiltAngle(self):
        self.accel.getAccelValues()
        return self.accel.calculateTilt()

    def calculateSpecificGravity(self):
        pass

    def measure_all(self):
        temp = self.getTemperature()
        volt = self.getBatteryVoltage()
        tilt = self.getTiltAngle()
        return {'temp': temp, 'tilt': tilt, 'battery': volt}

    def send2Ubidots(self, variable_label, value):
        #var_id = self.var_ids[variable_label]
        #print(var_id)
        self.ubi.send(variable_label, value)

    def sendAll2Ubidots(self, var_dict):
        self.ubi.send_all(var_dict)

    def deepSleep(self, sleep_time):
        self.sleep_timer.alarm(machine.RTC.ALARM0, sleep_time)
        self.sleep_timer.irq(trigger=machine.RTC.ALARM0, wake=machine.DEEPSLEEP)
        machine.deepsleep()

    def run(self):
        self.sleep_timer.alarm(machine.RTC.ALARM0, SLEEP_TIME)

    # Deep sleep mode
    """
    # configure RTC.ALARM0 to be able to wake the device
    rtc = machine.RTC()
    rtc.irq(trigger=rtc.ALARM0, wake=machine.DEEPSLEEP)

    # set RTC.ALARM0 to fire after 10 seconds (waking the device)
    rtc.alarm(rtc.ALARM0, 10000)

    # put the device to sleep
    machine.deepsleep()
    """

## Main
micropython.alloc_emergency_exception_buf(100)

drd = DoubleResetDetector(timeout=DBL_RST_TIMEOUT)

if machine.reset_cause() == machine.HARD_RESET:
    print('Hard Reset')
    if drd.detectDoubleReset():
        print('Double Reset')
        sys.exit()
        # Todo: Configuration script
        # Open config prompt
        #  1. Wifi settings
        #  2. Ubidots settings
        #  3. Calibrate Specific gravity settings
        #  4. Exit - To allow firmware updating via serial port

# check if the device woke from a deep sleep or other - mostly for debug
reset_cause = machine.reset_cause()
if reset_cause == machine.DEEPSLEEP_RESET:
    print('woke from a deep sleep')
elif reset_cause == machine.SOFT_RESET:
    print('woke from a soft reset')
    drd.clearRecentResetFlag()
    sys.exit() # for reprogramming
elif reset_cause == machine.PWRON_RESET:
    print('woke frome power on event')
    drd.clearRecentResetFlag()
    sys.exit()

## Init State
#do_connect()
sleep_timer = machine.RTC()
gy521 = GY521(I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ, accel_range=ACCEL_RANGE)
ds = ds18x20.DS18X20(onewire.OneWire(machine.Pin(DS18X20_PIN)))
battery_sensor = machine.ADC(ADC_PIN)

WW = WortWorm(gy521, ds, battery_sensor, UBI_DEVICE_LABEL, sleep_timer)

## Measure State
temp = WW.getTemperature()
print('Temp: %f' % temp)
tilt = WW.getTiltAngle()
print('Tilt: %f' % tilt)
volt = WW.getBatteryVoltage()
print('Battery Voltage: %f' % volt)
#sensor_values = WW.measure_all()

## Transmit State
#WW.sendAll2Ubidots(sensor_values)
WW.send2Ubidots(UBI_TEMP_LABEL, temp)
WW.send2Ubidots(UBI_TILT_LABEL, tilt)
WW.send2Ubidots(UBI_VOLT_LABEL, volt)

## Deep Sleep
if reset_cause == machine.HARD_RESET or reset_cause == machine.DEEPSLEEP_RESET:
    print('Going to Sleep..')
    WW.deepSleep(SLEEP_TIME)
