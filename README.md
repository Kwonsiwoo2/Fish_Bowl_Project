# 🛣️ Smart Fish Tank Management System

This project uses **STM32**, **Arduino**, and **Raspberry Pi** to create an automated fish tank management system. The system monitors parameters like turbidity, temperature, and water levels, displaying the data on an LCD screen and sending it to a server for remote monitoring and visualization via an Apache-based web interface.

---

## 🔄 Features

1. **Real-Time Monitoring**:
   - Monitors and displays:
     - **Turbidity** (water clarity)
     - **Temperature**
     - **Water Levels** (Level 1 and Level 2)

2. **Wi-Fi Connectivity**:
   - Connects to a Wi-Fi network using the ESP module.
   - Sends data to a Raspberry Pi server for logging and further analysis.

3. **LCD Display**:
   - Displays real-time data on a 16x2 LCD screen.
   - Provides immediate feedback on water conditions.

4. **Web-Based Visualization**:
   - Uses **Apache** and **SQL** on Raspberry Pi to visualize real-time and historical sensor data.
   - Provides graphs and tables to monitor turbidity, temperature, and water levels over time.

5. **Automated Alerts**:
   - Generates alerts when turbidity, temperature, or water levels exceed safe thresholds.
   - Alerts can be displayed on the LCD and logged on the server.

6. **Error Logging**:
   - Logs errors and abnormal conditions to the server for diagnostics and maintenance.

7. **Modular Code Structure**:
   - Easy to expand and modify for additional sensors or functionality.

---

## 🖼️ Project Images

### Real-Time Monitoring Display

![Real-Time Monitoring](./images/main.png)  
![Real-Time Monitoring](./images/apache.png)  

---

## 📊 System Architecture Diagram

### Diagram Overview

![System Diagram](./images/diagram.png)

---

## 🛠️ Hardware Used

- **STM32 Microcontroller**
- **Arduino with ESP8266/ESP32** for Wi-Fi
- **Raspberry Pi** as the server
- **LiquidCrystal I2C 16x2 LCD** for display
- **Turbidity Sensor**
- **Temperature Sensor**
- **Water Level Sensors**
- **Power Supply**

---

## 📚 Installation and Setup

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/yourusername/fish-tank-management.git
   cd fish-tank-management
   ```

2. **Arduino Code Setup**:
   - Open the Arduino code (e.g., `Main.ino`) in the Arduino IDE.
   - Install necessary libraries:
     ```bash
     WiFiEsp
     LiquidCrystal_I2C
     SoftwareSerial
     ```

3. **STM32 Setup**:
   - Flash the STM32 code using STM32CubeIDE or Keil.

4. **Raspberry Pi Server Setup**:
   - Install Apache and SQL on Raspberry Pi:
     ```bash
     sudo apt-get update
     sudo apt-get install apache2 mariadb-server php libapache2-mod-php php-mysql
     ```
   - Start Apache and SQL services:
     ```bash
     sudo systemctl start apache2
     sudo systemctl start mariadb
     ```
   - Place your PHP and HTML files in `/var/www/html` for visualization.

5. **Database Configuration**:
   - Create a database and tables to store the sensor data.
   - Import your SQL schema using:
     ```bash
     mysql -u root -p < schema.sql
     ```

---

## 📚 Project Structure

```
Fish_Tank_Project/
|
├── stm32/                  # STM32 source code
├── arduino/                # Arduino source code
│   └── Main.ino            # Main Arduino sketch
├── raspberry-pi/           # Raspberry Pi server code
│   ├── web/                # Apache web files (PHP, HTML, CSS)
│   └── database/           # SQL schema and scripts
└── README.md               # Project documentation
```

---

## 🌟 Contributing

Contributions are welcome! Feel free to submit issues or pull requests.

---

## 👇 Let's Connect!

[![GitHub](https://img.shields.io/badge/GitHub-Profile-blue?logo=github)](https://github.com/Kwonsiwoo2)  [![LinkedIn](https://img.shields.io/badge/LinkedIn-Profile-blue?logo=linkedin)](https://www.linkedin.com/in/%EC%8B%9C%EC%9A%B0-%EA%B6%8C-064765341/)
