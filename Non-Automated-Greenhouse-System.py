"""
This script is used for monitoring a greenhouse environment, collecting real-time data from sensors,
and logging all data locally and to the Cayenne IoT cloud for remote monitoring. 
This version does not control any actuators (e.g., fan, pump, or LED) like the automated system.

Key features include:

- **Sensors**:
  - DHT22 for temperature and humidity measurements.
  - TSL2591 for light intensity (lux) measurements.
  - DS18B20 for soil temperature measurements.
  - ADS1115 for soil moisture sensor readings.
  - SGP40 for VOC (Volatile Organic Compounds) index measurements.

- **Data Logging and Cloud Communication**:
  - Sensor data is transmitted to Cayenne IoT Cloud using MQTT for remote monitoring.
  - Data is also logged locally into a file with timestamps.

- **Sequential Execution**:
  Sensors are read sequentially, and data is processed and logged accordingly.
"""

import cayenne.client                # Import Cayenne client to send/receive data from Cayenne Cloud
import time                          # Importing time for delays and system operations
import adafruit_dht                  # Importing Adafruit's DHT library to use DHT22 sensor
import board                         # Importing board to use GPIO pins
import busio                         # Importing busio to use I2C pins for sensor communication
import adafruit_tsl2591              # Importing Adafruit's TSL2591 light sensor library
import os                            # Importing os for system-level operations
import adafruit_sgp40                # Importing Adafruit's SGP40 gas sensor
import glob                          # Importing glob to handle file paths for temperature sensor
import re                            # Importing regex module for text processing
from datetime import date, datetime  # Importing date and datetime for logging
from time import sleep               # Importing sleep for delays
import adafruit_ads1x15.ads1115 as ADS  # Importing Adafruit ADS1115 library
from adafruit_ads1x15.analog_in import AnalogIn  # Importing analog channel for sensor input

# Set up I2C bus for sensor communication
i2c = busio.I2C(board.SCL, board.SDA)

# Set up ADC (ADS1115) for soil moisture sensor
ads = ADS.ADS1115(i2c)
chan = AnalogIn(ads, ADS.P0)

# System setup for DS18B20 temperature sensor
os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')

# Initialize DHT22 for temperature and humidity readings
dht = adafruit_dht.DHT22(board.D17, use_pulseio=False)

# One-wire temperature sensor setup (DS18B20)
base_dir = '/sys/bus/w1/devices/'
device_folder = glob.glob(base_dir + '28*')[0]
device_file = device_folder + '/w1_slave'

# Initialize TSL2591 light sensor
sensor = adafruit_tsl2591.TSL2591(i2c)
sensor.gain = adafruit_tsl2591.GAIN_LOW

# Initialize SGP40 gas sensor
sgp = adafruit_sgp40.SGP40(i2c)

# Cayenne IoT Cloud credentials
MQTT_USERNAME = "59afff80-e844-11ed-9ab8-d511caccfe8c"
MQTT_PASSWORD = "31ca2453359a3bc83e7f2e36238a5aa9e953f6e6"
MQTT_CLIENT_ID = "65fbbbd0-e844-11ed-8485-5b7d3ef089d0"

# Initialize Cayenne client for cloud communication
client = cayenne.client.CayenneMQTTClient()
try:
    client.begin(MQTT_USERNAME, MQTT_PASSWORD, MQTT_CLIENT_ID)
except:
    today = date.today()
    timeToday = today.strftime("%d-%m-%Y")
    filename = f"/home/pi/Desktop/Data/{timeToday}.txt"
    with open(filename, 'a+') as file:
        timeNow = re.sub(' +', ' ', time.ctime())
        file.write(f"\n{timeNow} couldn't connect to the internet, rebooting Raspberry Pi")
        file.flush()
    time.sleep(60)
    os.system("sudo reboot")


def get_dhtData():
    """ Retrieves data from DHT22 and returns temperature in Celsius/Fahrenheit and humidity. """
    temperature_c = dht.temperature
    humidity = dht.humidity
    if humidity is not None and temperature_c is not None:
        if -40 <= temperature_c <= 80 and 0 <= humidity <= 100:
            temperature_f = temperature_c * 9 / 5 + 32
            return round(temperature_c, 2), round(temperature_f, 2), round(humidity, 2)
        else:
            print("Temperature or humidity is out of range")
            return 0, 0, 0
    else:
        print("Failed to retrieve data from DHT22 sensor")
        return 0, 0, 0


def get_luxData():
    """ Retrieves light intensity data from the TSL2591 sensor. """
    lux = sensor.lux
    if 0 <= lux <= 88000:
        return round(lux, 2)
    else:
        print("TSL2591 sensor data out of range")
        return 0


def map_to_percentage(value):
    """ Maps the ADS1115 sensor voltage to a soil moisture percentage. """
    if value <= 0.96:
        return 100.0
    elif value >= 1.6:
        return 0.0
    else:
        return 100 - ((value - 0.96) / (1.6 - 0.96)) * 100


def get_soilMoistureData():
    """ Retrieves soil moisture data from ADS1115 by averaging over multiple readings. """
    num_readings = 10
    total_touch = 0
    for i in range(num_readings):
        touch = chan.voltage
        total_touch += touch
        time.sleep(0.5)
    average_touch = total_touch / num_readings
    return round(map_to_percentage(average_touch), 2)


def get_sgpData():
    """ Retrieves raw VOC index data from the SGP40 gas sensor. """
    gas = sgp.raw
    if gas is not None:
        return gas
    else:
        print("Failed to retrieve data from SGP40")
        return 0


def read_temp_raw():
    """ Reads raw data from DS18B20 temperature sensor. """
    try:
        with open(device_file, 'r') as f:
            lines = f.readlines()
        return lines
    except Exception as e:
        print(f"Error reading DS18B20 sensor: {e}")
        return []


def read_temp():
    """ Processes raw data from DS18B20 and returns temperature in Celsius and Fahrenheit. """
    lines = read_temp_raw()
    if lines:
        while lines[0].strip()[-3:] != 'YES':
            time.sleep(0.2)
            lines = read_temp_raw()
        equals_pos = lines[1].find('t=')
        if equals_pos != -1:
            temp_c = float(lines[1][equals_pos + 2:]) / 1000.0
            if -10 <= temp_c <= 85:
                temp_f = temp_c * 9.0 / 5.0 + 32.0
                return round(temp_c, 2), round(temp_f, 2)
            else:
                print("DS18B20 sensor data out of range")
                return 0, 0
    print("Failed to retrieve data from DS18B20 sensor")
    return 0, 0


today = date.today()
timeToday = today.strftime("%d-%m-%Y")
filename = f"/home/pi/Desktop/Data/{timeToday}.txt"
with open(filename, 'a+') as file:
    timeNow = re.sub(' +', ' ', time.ctime())
    file.write(f"\n{timeNow} code restarted")
    file.flush()


while True:
    client.loop()  # Keep the Cayenne client running

    try:
        # Retrieve sensor data
        airTemperature_c, airTemperature_f, airHumidity = get_dhtData()
        lux_value = get_luxData()
        soilMoisture_value = get_soilMoistureData()
        soilTemperature_c, soilTemperature_f = read_temp()
        voc_index = sgp.measure_index(temperature=airTemperature_c, relative_humidity=airHumidity)

        # Log data locally
        with open(filename, 'a+') as file:
            timeNow = re.sub(' +', ' ', time.ctime())
            file.write(f"\n{timeNow} {airTemperature_c} {airTemperature_f} {airHumidity} {lux_value} "
                       f"{soilMoisture_value} {soilTemperature_c} {soilTemperature_f} {voc_index}")
            file.flush()

        # Send data to Cayenne IoT Cloud
        client.celsiusWrite(1, airTemperature_c)
        client.fahrenheitWrite(2, airTemperature_f)
        client.virtualWrite(3, airHumidity)
        client.luxWrite(4, lux_value)
        client.virtualWrite(5, soilMoisture_value)
        client.celsiusWrite(6, soilTemperature_c)
        client.fahrenheitWrite(7, soilTemperature_f)
        client.virtualWrite(8, voc_index)

        time.sleep(60)

    except Exception as e:
        print(f"Main Loop Failed, so trying again: {e}")
        time.sleep(1)  # Wait for a second before retrying
