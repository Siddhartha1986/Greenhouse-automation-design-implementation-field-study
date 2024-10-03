"""
This script is used for automating a greenhouse environment, 
controlling the fan, pump, and LED light based on real-time data 
collected from sensors. 
It also captures images at specific times and logs all data both 
locally and to the Cayenne IoT cloud for monitoring. 

The key features include:

- Microprocessor: Raspberry Pi 4 Model B

- **Sensors**:
  - DHT22 for temperature and humidity measurements.
  - TSL2591 for light intensity (lux) measurements.
  - DS18B20 for soil temperature measurements.
  - ADS1115 for soil moisture sensor readings.
  - SGP40 for VOC (Volatile Organic Compounds) index measurements.

- **Actuators**:
  - Fan control (Relay1) based on temperature setpoints.
  - Pump control (Relay3) based on soil moisture levels.
  - LED control (Relay2) based on light intensity and time of day.

- **Data Logging and Cloud Communication**:
  - Sensor data is transmitted to Cayenne IoT Cloud using MQTT for remote monitoring.
  - Data is also logged locally into a file with timestamps.

- **Image Capture**:
  - PiCamera captures images between 9 AM and 5 PM and saves them with timestamps.

- **Sequential Execution**:
  I chose to keep the system sequential because the fan process directly 
  influences the air temperature, which in turn affects the soil temperature. 
  If the fan does not run, the soil could dry out more quickly, 
  leading to unnecessary water pumping and resource wastage. 
  By processing each sensor in sequence, 
  I ensure that the system only moves forward once 
  critical environmental conditions—like air temperature—are stabilized. 
  This approach helps conserve water and maintain overall environmental balance.

"""




import cayenne.client  # Import Cayenne client to send/receive data from Cayenne Cloud
import time  # Importing time for delays and system operations
import adafruit_dht  # Importing Adafruit's DHT library to use DHT22 sensor
import board  # Importing board to use GPIO pins
import busio  # Importing busio to use I2C pins for sensor communication
import adafruit_tsl2591  # Importing Adafruit's TSL2591 light sensor library
from gpiozero import LED  # Importing gpiozero for controlling relays (LED, Fan, Pump)
import os  # Importing os for system-level operations
import adafruit_sgp40  # Importing Adafruit's SGP40 gas sensor
import glob  # Importing glob to handle file paths for temperature sensor
import re  # Importing regex module for text processing
from datetime import date, datetime  # Importing date and datetime for logging
from picamera import PiCamera  # Importing PiCamera for image capture
from time import sleep  # Importing sleep for delays
from multiprocessing import Process  # Importing Process for parallel execution
import adafruit_ads1x15.ads1115 as ADS  # Importing ADC for analog sensor reading
from adafruit_ads1x15.analog_in import AnalogIn  # Importing analog channel for sensor input

# Set up I2C bus for sensor communication
i2c = busio.I2C(board.SCL, board.SDA)

# Set up ADC for soil moisture sensor
ads = ADS.ADS1115(i2c)
chan = AnalogIn(ads, ADS.P0)

# Set up Pi Camera for image capture
camera = PiCamera()

# System setup for temperature sensors (DS18B20)
os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')

# Set up DHT22 for temperature and humidity readings
dht = adafruit_dht.DHT22(board.D17, use_pulseio=False)

# One-wire temperature sensor setup (DS18B20)
base_dir = '/sys/bus/w1/devices/'
device_folder = glob.glob(base_dir + '28*')[0]
device_file = device_folder + '/w1_slave'

# Relay setup (mapped to GPIO pins on Raspberry Pi)
Relay2 = LED(22)  # LED (Relay2) connected to pin 22
Relay1 = LED(23)  # Fan (Relay1) connected to pin 23
Relay3 = LED(24)  # Pump (Relay3) connected to pin 24

# Set up TSL2591 light sensor and configure gain
sensor = adafruit_tsl2591.TSL2591(i2c)
sensor.gain = adafruit_tsl2591.GAIN_LOW

# Set up SGP40 gas sensor
sgp = adafruit_sgp40.SGP40(i2c)

# Setpoints for controlling the actuators
airTemperatureSetpoint = 26  # Air temperature setpoint for fan control
moistureSetPoint = 65        # Soil moisture setpoint for pump control
luxSetPoint = 5000           # Lux setpoint for LED control

# Cayenne IoT Cloud credentials
MQTT_USERNAME = "41784500-e842-11ed-9ab8-d511caccfe8c"
MQTT_PASSWORD = "0b6719edec6ff4be34c13d3e6ed0cf6239dd350c"
MQTT_CLIENT_ID = "4f4215d0-e842-11ed-8485-5b7d3ef089d0"

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

# Function to get temperature and humidity data from DHT22 sensor
def get_dhtData():
    """
    Retrieves data from the DHT22 sensor, checks validity, and returns temperature and humidity.
    Returns:
        (float, float, float): temperature in Celsius, Fahrenheit, and humidity percentage.
    """
    temperature_c = dht.temperature
    humidity = dht.humidity

    if humidity is not None and temperature_c is not None:
        if -40 <= temperature_c <= 80 and 0 <= humidity <= 100:
            temperature_f = temperature_c * 9 / 5 + 32
            return round(temperature_c, 2), round(temperature_f, 2), round(humidity, 2)
        else:
            print("DHT22 temperature or humidity out of range.")
            return 0, 0, 0
    else:
        print("Failed to retrieve data from DHT22 sensor.")
        return 0, 0, 0

# Function to get lux data from the TSL2591 sensor
def get_luxData():
    """
    Retrieves light intensity data from the TSL2591 sensor.
    Returns:
        float: light level in lux.
    """
    lux = sensor.lux
    if 0 <= lux <= 88000:
        return round(lux, 2)
    else:
        print("TSL2591 lux out of range.")
        return 0

# Function to map soil moisture voltage reading to percentage
def map_to_percentage(value):
    """
    Maps the voltage value to a percentage for soil moisture.
    Args:
        value (float): Voltage reading from the sensor.
    Returns:
        float: Mapped percentage of soil moisture.
    """
    if value <= 0.96:
        return 100.0
    elif value >= 1.6:
        return 0.0
    else:
        return 100 - ((value - 0.96) / (1.6 - 0.96)) * 100

# Function to get soil moisture data from ADS1115 sensor
def get_soilMoistureData():
    """
    Reads soil moisture data from the ADS1115 sensor and averages over multiple readings.
    Returns:
        float: Average soil moisture percentage.
    """
    num_readings = 10
    total_touch = 0

    for i in range(num_readings):
        total_touch += chan.voltage
        time.sleep(0.5)

    average_touch = total_touch / num_readings
    return round(map_to_percentage(average_touch), 2)

# Function to get gas sensor data (VOC index) from SGP40
def get_sgpData():
    """
    Retrieves raw gas sensor data from the SGP40.
    Returns:
        float: VOC index.
    """
    gas = sgp.raw
    if gas is not None:
        return gas
    else:
        print("Failed to retrieve data from SGP40 sensor.")
        return 0

# Function to read raw data from DS18B20 temperature sensor
def read_temp_raw():
    """
    Reads raw data from the DS18B20 temperature sensor.
    Returns:
        list: Raw temperature data.
    """
    try:
        with open(device_file, 'r') as f:
            lines = f.readlines()
        return lines
    except Exception as e:
        print(f"Error reading DS18B20 sensor: {e}")
        return []

# Function to process raw temperature data from DS18B20
def read_temp():
    """
    Processes raw data from DS18B20 and returns the temperature in Celsius and Fahrenheit.
    Returns:
        (float, float): Temperature in Celsius and Fahrenheit.
    """
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
                print("DS18B20 temperature out of range.")
                return 0, 0
    print("Failed to retrieve data from DS18B20 sensor.")
    return 0, 0

# Function to control the fan based on temperature setpoint
def runFan():
    """
    Controls the fan (relay1) based on the air temperature setpoint.
    """
    if airTemperature_c > airTemperatureSetpoint:
        Relay1.off()  # Turns the fan on
    else:
        Relay1.on()  # Turns the fan off

# Function to control the pump based on soil moisture setpoint
def runPump():
    """
    Controls the pump (relay3) based on the soil moisture setpoint.
    """
    if soilMoisture_value < moistureSetPoint:
        Relay3.off()  # Turns the pump on
    else:
        Relay3.on()  # Turns the pump off

# Function to control LED light based on time and lux value
def runLED():
    """
    Controls the LED (relay2) based on the time of day and light intensity (lux).
    """
    now = datetime.now()
    hour_24 = int(now.strftime("%H"))

    if 9 <= hour_24 < 17:  # Between 9 AM and 5 PM
        if lux_value < luxSetPoint:
            Relay2.off()  # Turns the LED on
        else:
            Relay2.on()  # Turns the LED off

# Function to capture image using PiCamera between 9 AM and 5 PM
def capture_image():
    """
    Captures an image between 9 AM and 5 PM.
    The image is saved with a timestamp in the filename.
    """
    now = datetime.now()
    hour_24 = int(now.strftime("%H"))
    
    if 9 <= hour_24 < 17:  # Between 9 AM and 5 PM
        print("The current hour is between 9 AM and 5 PM.")
        today = date.today()
        timeToday = today.strftime("%d-%m-%Y")
        imageName = f"/home/pi/Desktop/Data/{timeToday}_{hour_24}.jpg"
        
        camera.start_preview()
        sleep(5)
        camera.capture(imageName)
        camera.stop_preview()
        print(f"Image captured and saved as {imageName}")
    else:
        print("The current hour is NOT between 9 AM and 5 PM. Image not captured.")

# Initialize all relays to off
Relay1.on()
Relay2.on()
Relay3.on()

# Main loop for data processing and control
while True:
    # Start parallel processes for running fan, pump, and LED control
    p1 = Process(target=runFan)  # Process for controlling the fan
    p2 = Process(target=runPump)  # Process for controlling the pump
    p3 = Process(target=runLED)  # Process for controlling the LED

    try:
        # Keeps Cayenne client connected and syncing with the cloud
        client.loop()

    except:
        # If there's an error connecting to Cayenne IoT, log it and reboot the Raspberry Pi
        today = date.today()
        timeToday = today.strftime("%d-%m-%Y")
        filename = f"/home/pi/Desktop/Data/{timeToday}.txt"
        with open(filename, 'a+') as file:
            timeNow = re.sub(' +', ' ', time.ctime())
            file.write(f"\n{timeNow} couldn't connect to the internet, rebooting Raspberry Pi")
            file.flush()
        time.sleep(60)  # Wait for 60 seconds before rebooting
        os.system("sudo reboot")  # Reboot the Raspberry Pi

    try:
        # Retrieve sensor data
        airTemperature_c, airTemperature_f, airHumidity = get_dhtData()  # Get temperature and humidity from DHT22
        soilMoisture_value = get_soilMoistureData()  # Get soil moisture data
        lux_value = get_luxData()  # Get light intensity (lux) data
        soilTemperature_c, soilTemperature_f = read_temp()  # Get soil temperature from DS18B20
        voc_index = get_sgpData()  # Get VOC index from SGP40 gas sensor

        # Send data to Cayenne IoT Cloud
        client.celsiusWrite(1, airTemperature_c)  # Sending air temperature in Celsius to Cayenne (channel 1)
        client.fahrenheitWrite(2, airTemperature_f)  # Sending air temperature in Fahrenheit to Cayenne (channel 2)
        client.virtualWrite(3, airHumidity)  # Sending air humidity to Cayenne (channel 3)
        client.luxWrite(4, lux_value)  # Sending light intensity (lux) to Cayenne (channel 4)
        client.virtualWrite(5, soilMoisture_value)  # Sending soil moisture data to Cayenne (channel 5)
        client.celsiusWrite(6, soilTemperature_c)  # Sending soil temperature in Celsius to Cayenne (channel 6)
        client.fahrenheitWrite(7, soilTemperature_f)  # Sending soil temperature in Fahrenheit to Cayenne (channel 7)
        client.virtualWrite(8, voc_index)  # Sending VOC index to Cayenne (channel 8)

        # Log and save sensor data locally into a file
        today = date.today()
        timeToday = today.strftime("%d-%m-%Y")
        filename = f"/home/pi/Desktop/Data/{timeToday}.txt"
        with open(filename, 'a+') as file:
            timeNow = re.sub(' +', ' ', time.ctime())
            file.write(f"\n{timeNow} {airTemperature_c} {airTemperature_f} {airHumidity} {lux_value} {soilMoisture_value} {soilTemperature_c} {soilTemperature_f} {voc_index}")
            file.flush()

        # Start and join parallel processes for controlling fan, pump, and LED
        p1.start()
        p1.join()  # Ensures the fan control process finishes before moving on
        p2.start()
        p2.join()  # Ensures the pump control process finishes
        p3.start()
        p3.join()  # Ensures the LED control process finishes

        # Capture image between 9 AM and 5 PM
        capture_image()

        # Wait for 60 seconds before running the loop again
        time.sleep(60)

    except Exception as e:
        # If there's an error, print the error message and reset the light sensor gain
        print(f"Error: {str(e)}. Retrying...")
        sensor.gain = adafruit_tsl2591.GAIN_LOW
        time.sleep(1)  # Pause for 1 second before retrying
