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

# Create the I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Create the ADC object using the I2C bus
ads = ADS.ADS1115(i2c)

# Create single-ended input on channel 0
chan = AnalogIn(ads, ADS.P0)

camera = PiCamera()
os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')
 
dht = adafruit_dht.DHT22(board.D17, use_pulseio=False)

base_dir = '/sys/bus/w1/devices/'
device_folder = glob.glob(base_dir + '28*')[0]
device_file = device_folder + '/w1_slave'

#variables for relay inputs, mapping them to gpio pins on raspberry pi
Relay2 = LED(22) #LED real 22
Relay1 = LED(23) #FAN real 23
Relay3 = LED(24) #PUMP real 24

# Create sensor object, communicating over the board's default I2C bus
i2c = busio.I2C(board.SCL, board.SDA)  # uses board.SCL and board.SDA
# Initialize the sensor.
sensor = adafruit_tsl2591.TSL2591(i2c)
sensor.gain = adafruit_tsl2591.GAIN_LOW
sgp = adafruit_sgp40.SGP40(i2c)


# Define the desired temperature range and PID constants
airTemperatureSetpoint = 26
Kp = 15.0  # Proportional gain
Ki = 0.0  # Integral gain
Kd = 0.0  # Derivative gain

# Define initial variables for PID controller
prev_error = 0
integral = 0

# Define the time interval for turning on and off the AC exhaust
time_interval = 60  # 1 minute

luxSetPoint = 5000
# Define the desired temperature range and PID constants
moistureSetPoint = 65
Kps = 5.0  # Proportional gain
Kis = 0.0  # Integral gain
Kds = 0.0  # Derivative gain

# Define initial variables for PID controller
prev_error_s = 0
integral_s = 0

# Define the time interval for turning on and off the pump
time_interval_s = 60  # 1 minute

MQTT_USERNAME  = "41784500-e842-11ed-9ab8-d511caccfe8c"      #mqtt username of IoT cloud
MQTT_PASSWORD  = "0b6719edec6ff4be34c13d3e6ed0cf6239dd350c"  #mqtt password of IoT cloud
MQTT_CLIENT_ID = "4f4215d0-e842-11ed-8485-5b7d3ef089d0"      #mqtt client id of IoT cloud

client = cayenne.client.CayenneMQTTClient()                  #declaring client from the cayenne library class
try:
    client.begin(MQTT_USERNAME, MQTT_PASSWORD, MQTT_CLIENT_ID)   #starting communication from the cayenne IoT cloud using the above mqtt credentials
except:
    today = date.today()
    timeToday = today.strftime("%d-%m-%Y")
    filename = "/home/pi/Desktop/Data/"+timeToday+'.txt'
    with open(filename, 'a+') as file:
        timeNow = re.sub(' +', ' ', time.ctime())
        file.write("\n"+timeNow+" "+ "couldn't connect to the internet, rebooting raspberry pi")
        file.flush()
        print('data updated')
        print('=====================================')

    time.sleep(60)
    os.system("sudo reboot")


def get_dhtData():   #function which returns data obtained from dht11
        temperature_c = dht.temperature
        humidity = dht.humidity         #getting temperature data from dht11 #getting humidity data from dht11
         
        if humidity is not None and temperature_c is not None:
            # Check if temperature data is within the valid range of -40 to 80 degrees Celsius
            if temperature_c >= -40 and temperature_c <= 80:
                # Check if humidity data is within the valid range of 0 to 100 percent
                if humidity >= 0 and humidity <= 100:
                    temperature_f = temperature_c * (9 / 5) + 32  #converting centrigrade to fahrenheit 
                    temperature_c = round(temperature_c, 2)       #rounding the float value to 2 decimal points
                    temperature_f = round(temperature_f, 2)       #rounding the float value to 2 decimal points
                    humidity = round(humidity, 2)                 #rounding the float value to 2 decimal points
                    return temperature_c, temperature_f, humidity #sending back the data
                else:
                    print("Humidity data is out of range")
                    return 0, 0, 0
            else:
                print("Temperature data is out of range")
                return 0, 0, 0
        else:
            print("Failed to retrieve data from sensor")
            return 0, 0, 0      

def get_luxData():   #functions which returns the lux value
    # Read and calculate the light level in lux.
    lux = sensor.lux  #asking lux data from sensor
    # Check if the lux value is between 0 and 88000
    if 0 <= lux <= 88000:
        print("TSL2591 is giving data between 0 - 88000 lux")
        lux = round(lux, 2) #rounding the float value to 2 decimal points
        return lux  #sending back the data
    else:
        print("TSL2591 is not giving data between 0 - 88000 lux")
        return 0

def map_to_percentage(value):
    if value <= 0.96:
        return 100.0
    elif value >= 1.6:
        return 0.0
    else:
        percentage = 100 - ((value - 0.96) / (1.6 - 0.96)) * 100
        return percentage
def get_soilMoistureData():   #functions which returns the soil moisture value from ads1115

    num_readings = 10

    total_touch = 0

    for i in range(num_readings):
        # read moisture level through capacitive touch pad
        touch = chan.voltage
        total_touch += touch

        time.sleep(0.5)

    average_touch = total_touch / num_readings
    average_touch = map_to_percentage(average_touch)
    average_touch = round(average_touch, 2)
    print("moisture: " + str(average_touch))
    return average_touch #sending back the data

def get_sgpData():
    gas = sgp.raw
    return gas

def read_temp_raw():
    f = open(device_file, 'r')
    lines = f.readlines()
    f.close()
    return lines

def read_temp():
    lines = read_temp_raw()
    while lines[0].strip()[-3:] != 'YES':
        time.sleep(0.2)
        lines = read_temp_raw()
    equals_pos = lines[1].find('t=')
    if equals_pos != -1:
        temp_string = lines[1][equals_pos+2:]
        temp_c = float(temp_string) / 1000.0
        if temp_c >= -10 and temp_c <= 85: 
            print("DS18B20 is giving data between -10 and 85")
            temp_c = round(temp_c, 2)
            temp_f = temp_c * 9.0 / 5.0 + 32.0
            temp_f = round(temp_f, 2)
            return temp_c, temp_f
        else:
            print("DS18B20 is not giving data between -10 and 85")
            return 0, 0

    
def PIDController(data):
    global error,P,integral,derivative,cooling_value,prev_error,time_interval
    # Read the current temperature value
    temperature = data
    
    # Calculate the error signal
    error = temperature - airTemperatureSetpoint

    # Calculate the proportional term
    P = Kp * error
    # Calculate the integral term
    integral += Ki * error
    # Calculate the derivative term
    derivative = Kd * (error - prev_error)
    # Calculate the control input (cooling value)
    cooling_value = P + integral + derivative

    # Limit the cooling value to stay within lower and upper limits
    cooling_value = max(0, min(cooling_value, 100))

    # Calculate the time duration for turning on the AC exhaust based on the cooling value

    on_duration = int(cooling_value / 100 * time_interval)
    
    # Store the current error for the next iteration
    prev_error = error

    return cooling_value, on_duration

def PIDControllerSOilMoisture(data):
    global error_s,P_s,integral_s,derivative_s,pump_value,prev_error_s,time_interval_s
    # Read the current temperature value
    moisture = data
    
    # Calculate the error signal
    error_s = moistureSetPoint - moisture

    # Calculate the proportional term
    P_s = Kps * error_s
    # Calculate the integral term
    integral_s += Kis * error_s
    # Calculate the derivative term
    derivative_s = Kds * (error_s - prev_error_s)
    # Calculate the control input (cooling value)
    pump_value = P_s + integral_s + derivative_s
    # Limit the cooling value to stay within lower and upper limits
    pump_value = max(0, min(pump_value, 100))

    # Calculate the time duration for turning on the AC exhaust based on the cooling value

    on_duration_s = int(pump_value / 100 * time_interval_s)
    
    # Store the current error for the next iteration
    prev_error_s = error_s

    return pump_value, on_duration_s

def runFan():

    if airTemperature_c > airTemperatureSetpoint:
        Relay1.off() #turns relay2 on
        print("Fan is running")
    else:
        Relay1.on()  #turns relay2 off
        print("Fan has stopped")
        
        

def runPump():

    if soilMoisture_value < moistureSetPoint:
        Relay3.off() #turns relay2 on
        print("PUMP is running")
    else:
        Relay3.on()  #turns relay2 off
        print("PUMP has stopped")
    

def runLED():
    now = datetime.now()
    hour_24 = int(now.strftime("%H"))

    if hour_24 >= 9 and hour_24 < 17:
        print("The current hour is between 9 AM and 5 PM.")
        if lux_value < luxSetPoint:
            Relay2.off() #turns relay2 on
            print("LED is running")
        else:
            Relay2.on()  #turns relay2 off
            print("LED has stopped")
          

    

Relay1.on() #turining first relay off
Relay2.on() #turining second relay off
Relay3.on() #turining third relay off

today = date.today()
timeToday = today.strftime("%d-%m-%Y")
filename = "/home/pi/Desktop/Data/"+timeToday+'.txt'
with open(filename, 'a+') as file:
    timeNow = re.sub(' +', ' ', time.ctime())
    file.write("\n"+timeNow+" "+ "code restarted")
    file.flush()
    print('data updated')
    print('=====================================')


while True:
    p1 = Process(target=runFan, args=())
    p2 = Process(target=runPump, args=())
    p3 = Process(target = runLED, args = ())

    try:
        client.loop()  #cloud is keept in loop to sync

    except:
        with open(filename, 'a+') as file:
            timeNow = re.sub(' +', ' ', time.ctime())
            file.write("\n"+timeNow+" "+ "couldn't connect to the internet, rebooting raspberry pi")
            file.flush()
            print('data updated')
            print('=====================================')

        time.sleep(60)
        os.system("sudo reboot")
    try:   #try block
        airTemperature_c,airTemperature_f, airHumidity = get_dhtData()  #asking data from get_dhtData() function
        print("air sensor",airTemperature_c,airTemperature_f,airHumidity)
        coolingValue,onDuration = 0,0
        # coolingValue,onDuration = PIDController(airTemperature_c)
        # print("cooling pid value is",coolingValue,onDuration)
        p1.start()
        p1.join()
        soilMoisture_value = get_soilMoistureData()  #asking data from get_soilMoistureData() function
        print("soilMoisture",soilMoisture_value)
        pumpingValue, onDuration_s = 0, 0
        # pumpingValue, onDuration_s = PIDControllerSOilMoisture(soilMoisture_value)
        # print("pumping pid value is",pumpingValue,onDuration_s)
        p2.start()
        p2.join()
        lux_value = get_luxData() #asking data from get_luxData() function
        print("lux",lux_value)
        p3.start()
        p3.join()
        soilTemperature_c, soilTemperature_f = read_temp()
        print("soilTemperature",soilTemperature_c,soilTemperature_f)
        voc_index = sgp.measure_index(temperature=airTemperature_c, relative_humidity=airHumidity)
        print("VOC Index:", voc_index)

        today = date.today()
        timeToday = today.strftime("%d-%m-%Y")
        filename = "/home/pi/Desktop/Data/"+timeToday+'.txt'
        now = datetime.now()
        hour_24 = int(now.strftime("%H"))

        if hour_24 >= 9 and hour_24 < 17:
            print("The current hour is between 9 AM and 5 PM.")
            imageName = "/home/pi/Desktop/Data/"+timeToday+" "+str(hour_24)+'.jpg'
            camera.start_preview()
            sleep(5)
            camera.capture(imageName)
            camera.stop_preview()
            print("Image captured and saved")
        else:
            print("The current hour is NOT between 9 AM and 5 PM. Hence image not captured")


        with open(filename, 'a+') as file:
            timeNow = re.sub(' +', ' ', time.ctime())
            file.write("\n"+timeNow+" "+str(airTemperature_c)+" "+str(airTemperature_f)+" "+str(airHumidity)+" "+str(lux_value)+" "+str(soilMoisture_value)+" "+str(soilTemperature_c)+" "+str(soilTemperature_f)+" "+str(voc_index)+" "+str(coolingValue)+" "+str(pumpingValue))
            file.flush()
            print('data updated')
            print('=====================================')

        client.celsiusWrite(1, airTemperature_c)  #sending celsius data to 1 number channel in cloud
        client.fahrenheitWrite(2, airTemperature_f) #sending fahrenheit data to 2 number channel in cloud
        client.virtualWrite(3, airHumidity)  #sending humidity data to 3 number channel in cloud
        client.luxWrite(4, lux_value)  #sending lux data to 4 number channel in cloud
        client.virtualWrite(5, soilMoisture_value) #sending soil moisture data to 5 number channel in cloud
        client.celsiusWrite(6, soilTemperature_c)
        client.fahrenheitWrite(7, soilTemperature_f)
        client.virtualWrite(8, voc_index) #sending soil moisture data to 5 number channel in cloud


        time.sleep(60)



    except: #if try fails
        print("Main Loop Failed, so trying again")  #prints message 
        sensor.gain = adafruit_tsl2591.GAIN_LOW
        time.sleep(1)  #halts for 1 second
