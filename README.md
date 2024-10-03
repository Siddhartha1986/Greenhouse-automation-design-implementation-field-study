# Greenhouse Automation and Monitoring

This repository contains code for the thesis ( project) for the automated greenhouse system and the non-automated system. Both were implemented as part of a larger research project aimed at creating an efficient, reliable, and cost-effective greenhouse solution tailored for greenhouse farming in central Nepal.

For more detailed background information, refer to the [Thesis Document](https://www.theseus.fi/handle/10024/812774).

## **Overview**

This project is the culmination of a **Bachelor of Engineering Thesis** from the **Metropolia University of Applied Sciences**, focusing on **Smart IoT Systems**. The main objective was to design and implement an automated greenhouse system to improve agricultural productivity by controlling key environmental factors such as temperature, humidity, soil moisture, and light. This repository also includes a non-automated system to compare the results.

### **Key Features**
- **Automated System:**
  - Regulates air temperature, soil moisture, and light using sensors and actuators (fan, water pump, and LED).
  - Collects real-time data using various sensors and sends it to the Cayenne IoT cloud for monitoring.
  - Includes a camera module for real-time image capture of the crops.
  
- **Non-Automated System:**
  - Collects sensor data (temperature, humidity, soil moisture) but does not control the environment.
  - Enables comparative analysis with the automated system.

## **Thesis Summary**

In underdeveloped countries like Nepal, traditional greenhouses often face challenges related to manual labor, resource inefficiencies, and unpredictable outcomes. This project aimed to develop a fully automated IoT-based greenhouse system and test its effectiveness in a real-world scenario in Panauti, Nepal. 

The main goal was to enhance productivity by controlling key parameters such as air temperature and soil moisture. A comparative study was conducted between an automated and a non-automated greenhouse, where crops were grown under similar conditions. 

### **Main Objectives**
1. Develop a low-cost, fully automated greenhouse system.
2. Compare its performance with a traditional, non-automated greenhouse.
3. Analyze the collected data to assess the improvements in crop growth and environmental regulation.

### **Results**
The automated system significantly improved crop growth by effectively maintaining stable environmental conditions. Energy consumption was minimal, costing just NPR 165 (1.13 Euros) per month, and plant growth was notably better in the automated system.

## **Systems Breakdown**
### **1. Automated Greenhouse System**
The automated system uses various sensors to monitor:
- **Air temperature and humidity:** Managed via the DHT22 sensor.
- **Soil moisture:** Measured using a capacitive soil moisture sensor.
- **Light intensity:** TSL2591 sensor monitors light.
- **Air quality:** Monitored by the SGP40 sensor.
- **Soil temperature:** Measured via DS18B20.
  
Control mechanisms include:
- **Fan:** For air temperature control.
- **Water Pump:** For drip irrigation based on soil moisture.
- **LED Strip:** For additional lighting if the lux levels drop below the threshold.

### **2. Non-Automated Greenhouse System**
This system collects data using the same sensors but without controlling the environment. It allows for a side-by-side comparison with the automated system to observe how active monitoring and control impact crop yield and growth.

### **Setup Overview**
Both systems were implemented in greenhouses set up in Panauti, Nepal. Data was logged over a 28-day period for analysis. The systems utilized **Raspberry Pi 4**, **Cayenne IoT cloud**, and various sensors to collect and monitor real-time data, which was logged both locally and in the cloud.

### **Results from the Study**
- **Air Temperature:** The automated system successfully maintained stable air temperatures, preventing harmful spikes.
- **Soil Moisture:** The drip irrigation system in the automated greenhouse maintained optimal soil moisture, whereas the non-automated system experienced significant fluctuations.
- **Light:** Both systems received sufficient light during daylight hours, so artificial lighting was rarely necessary.

## **Technologies Used**
- **Hardware:** Raspberry Pi 4, sensors (DHT22, DS18B20, TSL2591, ADS1115, SGP40), actuators (fans, pumps, LEDs).
- **Software:** Python (for sensor data collection and control), Cayenne IoT cloud (for real-time monitoring), GPIO library (for relay control).

## **Repository Contents**
- **Automated System Code:** Contains the full Python code for the automated greenhouse, including sensor integration and control logic.
- **Non-Automated System Code:** Provides the Python code for data collection without environmental control.
- **Thesis Document:** A PDF of the full thesis detailing the project, implementation, and analysis.

## **How to Use**
1. Clone the repository to your local machine:
   ```bash
   git clone https://github.com/yourusername/greenhouse-automation.git
   ```
2. Navigate to the appropriate folder (automated or non-automated) depending on your use case.
3. Install the necessary libraries:
   ```bash
   pip install -r requirements.txt
   ```
4. Connect the required hardware (Raspberry Pi, sensors, actuators).
5. Run the system by executing the Python scripts.

## **Acknowledgments**
This project was supervised by **Saana Vallius, Senior Lecturer at Metropolia University of Applied Sciences**, and conducted with support from local farmers in Panauti, Nepal.

## **License**
This project is licensed under the **MIT License**. See the `LICENSE` file for more details.

---


