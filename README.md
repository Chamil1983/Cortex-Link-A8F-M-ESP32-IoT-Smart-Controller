# Cortex Link A8F-M ESP32 IoT Controller

<p align="center">
<img src="3D_A8R-M PCB Layout_1.png" alt="Cortex Link A8R-M ESP32 Smart Relay Board" width="600"/>
</p>
![Cortex Link A8F-M ESP32 Board](Pictures/3D_A8F-M-I_IPEX PCB Layout TOP.png)

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![Arduino Compatible](https://img.shields.io/badge/Arduino-Compatible-green.svg)](https://www.arduino.cc/)
[![ESPHome Compatible](https://img.shields.io/badge/ESPHome-Compatible-green.svg)](https://esphome.io/)
[![Home Assistant Compatible](https://img.shields.io/badge/Home_Assistant-Compatible-green.svg)](https://www.home-assistant.io/)

## Overview

The Cortex Link A8F-M ESP32 is a high-performance, industrial-grade IoT controller designed for smart home automation, industrial monitoring, and IoT applications. Built around the powerful ESP32 dual-core processor, this controller offers extensive I/O capabilities, multiple communication interfaces, and seamless integration with popular development platforms.

Designed and manufactured by Microcode Embedded Systems and Automation (MESA), this controller provides a robust hardware platform for developing complex automation solutions with minimal additional components required.

## Features

- **Powerful Processing**: ESP32 dual-core 32-bit processor @ 240MHz with 8MB flash memory and 520KB SRAM
- **Extensive I/O**:
  - 16× MOSFET outputs (12/24V DC, 500mA per channel)
  - 8× optically isolated digital inputs
  - 4× analog inputs (2× 4-20mA, 2× 0-5V DC)
  - 2× analog outputs (0-10V)
- **Versatile Connectivity**:
  - Wi-Fi 802.11 b/g/n (2.4 GHz)
  - Bluetooth BLE 4.0 and Bluetooth Classic
  - Ethernet via RJ45 port (10/100 Mbps)
  - RS485/Modbus RTU interface
  - Optional GSM (2G/4G module support)
- **Additional Features**:
  - DS3231 real-time clock with battery backup
  - RF 433MHz/315MHz transmitter and receiver interface
  - Temperature/humidity sensor inputs
  - I2C expansion interface
  - 1-Wire support for temperature sensors

## Repository Contents

```
cortexlink-a8f-m/
├── docs/                   # Documentation
│   ├── images/             # Documentation images
│   ├── datasheets/         # Component datasheets
│   ├── user-manual.md      # User manual
│   └── pinout.md           # Detailed pin mappings
├── examples/               # Example code
│   ├── arduino/            # Arduino IDE examples
│   ├── esphome/            # ESPHome YAML configurations
│   └── micropython/        # MicroPython examples
├── hardware/               # Hardware design files
│   ├── schematics/         # Schematics in PDF format
│   └── gerber/             # Gerber files (if publicly available)
├── firmware/               # Firmware files
│   ├── factory/            # Factory default firmware
│   └── custom/             # Custom firmware examples
├── libraries/              # Custom libraries
│   ├── CortexLink/         # Core library for Arduino IDE
│   └── dependencies/       # Required third-party libraries
├── tools/                  # Useful tools and utilities
│   ├── i2c-scanner/        # I2C device scanner
│   ├── diagnostics/        # Diagnostic utilities
│   └── config-generator/   # Configuration file generator
├── LICENSE                 # License file
└── README.md               # This file
```

## Getting Started

### Prerequisites

- [Arduino IDE](https://www.arduino.cc/en/software) (1.8.x or higher)
- [ESP32 Arduino Core](https://github.com/espressif/arduino-esp32)
- Required libraries:
  - [Adafruit MCP23017 Library](https://github.com/adafruit/Adafruit-MCP23017-Arduino-Library)
  - [ModbusMaster](https://github.com/4-20ma/ModbusMaster)
  - [TinyGSM](https://github.com/vshymanskyy/TinyGSM) (for GSM functionality)
  - [ESPHome](https://esphome.io/) (optional, for Home Assistant integration)

### Hardware Setup

1. **Power Connection**
   - Connect 9-12V DC power supply to the power input terminals (observe polarity)
   - Verify the power LED illuminates

2. **Initial Connection**
   - Connect the board to your computer using a USB-B cable
   - If necessary, install the USB drivers for your operating system
   - Hold the BOOT button while pressing RESET to enter programming mode

3. **Input/Output Connections**
   - Connect inputs and outputs according to your application requirements
   - Refer to the [pinout documentation](docs/pinout.md) for detailed connection information

### Basic Arduino Setup

1. Install the Arduino IDE and ESP32 core
2. Clone or download this repository
3. Open Arduino IDE and set up the ESP32 board:
   - Board: "ESP32 Dev Module"
   - Upload Speed: 921600
   - Flash Frequency: 80MHz
   - Flash Mode: QIO
   - Flash Size: 8MB
   - Partition Scheme: Default
4. Open an example from the `examples/arduino/` directory
5. Modify the example to match your network settings and requirements
6. Upload the sketch to your Cortex Link board

### Basic ESPHome Setup

1. Install ESPHome:
   ```bash
   pip install esphome
   ```

2. Create a basic configuration file:
   ```yaml
   # cortexlink.yaml
   esphome:
     name: cortexlink
     platform: ESP32
     board: esp32dev

   wifi:
     ssid: "YourWiFiSSID"
     password: "YourWiFiPassword"
     
   api:
     password: "your_api_password"
     
   ota:
     password: "your_ota_password"

   i2c:
     sda: 21
     scl: 22
     scan: true
   
   # Basic configuration for I/O
   mcp23017:
     - id: input_expander
       address: 0x21
     - id: output_expander
       address: 0x20
   ```

3. Compile and upload:
   ```bash
   esphome run cortexlink.yaml
   ```

## Code Examples

### Basic I/O Control

```cpp
#include <Wire.h>
#include <Adafruit_MCP23017.h>

Adafruit_MCP23017 inputExpander;  // Digital inputs
Adafruit_MCP23017 outputExpander; // MOSFET outputs

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Initialize I/O expanders
  inputExpander.begin(0x21);  // Address for input expander
  outputExpander.begin(0x20); // Address for output expander
  
  // Configure inputs with pull-ups
  for (uint8_t i = 0; i < 8; i++) {
    inputExpander.pinMode(i, INPUT);
    inputExpander.pullUp(i, HIGH);
  }
  
  // Configure outputs
  for (uint8_t i = 0; i < 12; i++) {
    outputExpander.pinMode(i, OUTPUT);
    outputExpander.digitalWrite(i, LOW);
  }
  
  Serial.println("Cortex Link A8F-M initialized");
}

void loop() {
  // Read all digital inputs
  Serial.println("Digital Inputs:");
  for (uint8_t i = 0; i < 8; i++) {
    bool state = !inputExpander.digitalRead(i); // Inputs are active LOW
    Serial.print("Input ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(state ? "ON" : "OFF");
  }
  
  // Toggle outputs sequentially
  for (uint8_t i = 0; i < 12; i++) {
    outputExpander.digitalWrite(i, HIGH);
    Serial.print("Output ");
    Serial.print(i + 1);
    Serial.println(" ON");
    delay(500);
    outputExpander.digitalWrite(i, LOW);
    Serial.print("Output ");
    Serial.print(i + 1);
    Serial.println(" OFF");
    delay(500);
  }
}
```

### Reading Analog Inputs

```cpp
void setup() {
  Serial.begin(115200);
  
  // Configure ADC resolution
  analogReadResolution(12); // 12-bit resolution (0-4095)
}

void loop() {
  // Read 4-20mA inputs (GPIO34 & GPIO35)
  int current1Raw = analogRead(34);
  int current2Raw = analogRead(35);
  
  // Convert raw ADC values to current (mA)
  float current1 = map(current1Raw, 0, 4095, 4.0, 20.0);
  float current2 = map(current2Raw, 0, 4095, 4.0, 20.0);
  
  // Read 0-5V inputs (GPIO36 & GPIO39)
  int voltage1Raw = analogRead(36);
  int voltage2Raw = analogRead(39);
  
  // Convert raw ADC values to voltage
  float voltage1 = (voltage1Raw / 4095.0) * 5.0;
  float voltage2 = (voltage2Raw / 4095.0) * 5.0;
  
  // Display results
  Serial.println("Analog Inputs:");
  Serial.print("Current 1: "); Serial.print(current1); Serial.println(" mA");
  Serial.print("Current 2: "); Serial.print(current2); Serial.println(" mA");
  Serial.print("Voltage 1: "); Serial.print(voltage1); Serial.println(" V");
  Serial.print("Voltage 2: "); Serial.print(voltage2); Serial.println(" V");
  
  delay(1000);
}
```

### Wi-Fi Connection and MQTT Publishing

```cpp
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_MCP23017.h>

// Network and MQTT settings
const char* ssid = "YourWiFiSSID";
const char* password = "YourWiFiPassword";
const char* mqtt_server = "192.168.1.100";
const int mqtt_port = 1883;
const char* mqtt_user = "your_username";
const char* mqtt_password = "your_password";
const char* client_id = "cortexlink";

WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_MCP23017 inputExpander;

void setup_wifi() {
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(client_id, mqtt_user, mqtt_password)) {
      Serial.println("connected");
      client.subscribe("cortexlink/control/#");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 5 seconds");
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  // Handle incoming MQTT messages
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println(message);
  
  // Example: Control outputs via MQTT
  if (String(topic) == "cortexlink/control/output") {
    int output = message.substring(0, message.indexOf(':')).toInt();
    int state = message.substring(message.indexOf(':') + 1).toInt();
    
    if (output >= 1 && output <= 12) {
      outputExpander.digitalWrite(output - 1, state ? HIGH : LOW);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Initialize I/O expanders
  inputExpander.begin(0x21);
  outputExpander.begin(0x20);
  
  // Configure I/O
  for (uint8_t i = 0; i < 8; i++) {
    inputExpander.pinMode(i, INPUT);
    inputExpander.pullUp(i, HIGH);
  }
  
  for (uint8_t i = 0; i < 12; i++) {
    outputExpander.pinMode(i, OUTPUT);
    outputExpander.digitalWrite(i, LOW);
  }
  
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  // Read inputs and publish to MQTT
  for (uint8_t i = 0; i < 8; i++) {
    bool state = !inputExpander.digitalRead(i);
    String topic = "cortexlink/status/input" + String(i + 1);
    client.publish(topic.c_str(), state ? "ON" : "OFF");
  }
  
  // Read analog inputs
  int current1Raw = analogRead(34);
  float current1 = (current1Raw / 4095.0) * 16.0 + 4.0;
  client.publish("cortexlink/status/current1", String(current1).c_str());
  
  int voltage1Raw = analogRead(36);
  float voltage1 = (voltage1Raw / 4095.0) * 5.0;
  client.publish("cortexlink/status/voltage1", String(voltage1).c_str());
  
  delay(5000); // Publish every 5 seconds
}
```

## ESPHome Example Configuration

```yaml
# cortexlink-full.yaml
esphome:
  name: cortexlink
  platform: ESP32
  board: esp32dev

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password
  
  # Optional: Fallback AP
  ap:
    ssid: "Cortex Link Fallback"
    password: !secret ap_password

# Enable Home Assistant API
api:
  password: !secret api_password

ota:
  password: !secret ota_password

# Enable logging
logger:
  level: INFO

# Web server for device info
web_server:
  port: 80
  auth:
    username: admin
    password: !secret web_password

# I2C Bus
i2c:
  sda: 21
  scl: 22
  scan: true

# MCP23017 I/O Expanders
mcp23017:
  - id: input_expander
    address: 0x21
  - id: output_expander
    address: 0x20

# Digital Inputs
binary_sensor:
  - platform: gpio
    pin:
      mcp23017: input_expander
      number: 0
      inverted: true
    name: "Digital Input 1"
    device_class: motion
  # Repeat for inputs 2-8...

# MOSFET Outputs
switch:
  - platform: gpio
    pin:
      mcp23017: output_expander
      number: 0
    name: "MOSFET Output 1"
  # Repeat for outputs 2-16...

# Analog Inputs
sensor:
  - platform: adc
    pin: GPIO34
    name: "Analog Current 1"
    unit_of_measurement: "mA"
    accuracy_decimals: 2
    update_interval: 10s
    filters:
      - lambda: return (x * 3.3 / 4095.0) * 16.0 + 4.0;
  
  - platform: adc
    pin: GPIO36
    name: "Analog Voltage 1"
    unit_of_measurement: "V"
    accuracy_decimals: 2
    update_interval: 10s
    filters:
      - multiply: 5.0
      - divide: 4095.0

# System Sensors
  - platform: wifi_signal
    name: "WiFi Signal"
    update_interval: 60s
  
  - platform: uptime
    name: "Uptime"
    
# DHT22 Temperature/Humidity Sensor
  - platform: dht
    pin: GPIO4
    model: DHT22
    temperature:
      name: "Room Temperature"
    humidity:
      name: "Room Humidity"
    update_interval: 60s
```

## Project Structure and Development

### Core Library

The CortexLink library provides a simple interface to access all the features of the hardware:

```cpp
#include <CortexLink.h>

CortexLink controller;

void setup() {
  Serial.begin(115200);
  
  // Initialize with default settings
  if (!controller.begin()) {
    Serial.println("Failed to initialize Cortex Link hardware!");
    while (1);
  }
  
  // Configure I/O as needed
  controller.configureInput(1, INPUT_DIGITAL);
  controller.configureInput(3, INPUT_ANALOG_CURRENT);
  controller.configureOutput(1, OUTPUT_DIGITAL);
  
  Serial.println("Cortex Link initialized successfully.");
}

void loop() {
  // Read inputs
  bool input1 = controller.readDigitalInput(1);
  float current = controller.readAnalogCurrent(1);
  
  // Control outputs
  controller.writeDigitalOutput(1, input1);
  
  // Print values
  Serial.print("Input 1: ");
  Serial.println(input1 ? "ON" : "OFF");
  Serial.print("Current: ");
  Serial.print(current);
  Serial.println(" mA");
  
  delay(1000);
}
```

## Documentation

Complete documentation is available in the [docs](/docs) directory:

- [User Manual](docs/user-manual.md) - Comprehensive guide to hardware and software
- [Pinout Reference](docs/pinout.md) - Detailed pin mapping and descriptions
- [Library Reference](docs/library-reference.md) - API documentation for the CortexLink library
- [Application Notes](docs/application-notes/) - Specific use cases and solutions

## Contributing

Contributions to improve the Cortex Link A8F-M ESP32 project are welcome:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

Please ensure your code follows the existing style and includes appropriate documentation.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgements

- [Espressif Systems](https://www.espressif.com/) for the ESP32 platform
- [Adafruit Industries](https://www.adafruit.com/) for their excellent libraries
- [ESPHome](https://esphome.io/) for simplifying Home Assistant integration
- [Arduino](https://www.arduino.cc/) for their development platform

## Contact and Support

For technical support, please contact MESA:

- **Website:** [www.mesa-automation.com](https://www.mesa-automation.com)
- **Email:** support@mesa-automation.com

For issues related to this repository, please open an issue on GitHub.

---

*Designed and manufactured by Microcode Embedded Systems and Automation (MESA)*
