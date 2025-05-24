# AI-Augmented-Logistics-Transport-Monitoring-and-Control-IoT-System

This project presents a proof-of-concept (POC) for a modular, web-based logistics monitoring and control system integrating Internet of Things (IoT) sensing, Near Field Communication (NFC)-based security, and generative AI analytics. Developed as part of a university coursework module, it addresses operational challenges in Malaysia's logistics sector by providing real-time environmental monitoring, secure driver authentication, and AI-driven decision support. 

Please note that this serves as a display of concept, the functionality (e.g., security) are not meant to be production-ready. 

Watch a simple demo in Youtube:

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/hiIWA73AVaU/0.jpg)](https://www.youtube.com/watch?v=hiIWA73AVaU&ab_channel=ChorWT)

## System Architecture and features.

- **IoT Sensing**: Utilizes Arduino-based sensors to monitor temperature and humidity within transport vehicles.
- **Access Control**: Implements NFC-based driver authentication to ensure authorized access.
- **Data Transmission**: Employs MQTT and WebSockets protocols to transmit sensor data to a backend server.
- **Backend Processing**: A Flask–SocketIO server, developed in Python, handles real-time data processing and communication.
- **AI Analytics**: Integrates Google's Gemini large language model to generate intelligent status updates and provide dynamic environmental control recommendations.

## Repository Structure
├── arduino/ # Arduino firmware for sensor data acquisition and NFC authentication

├── python/ # Flask–SocketIO backend server code

├── LICENSE # MIT License

└── README.md # Project documentation

## Getting Started

1. **Hardware Setup**  
   Deploy Arduino-compatible microcontrollers equipped with temperature, humidity sensors, and NFC readers in transport vehicles.

2. **Firmware Deployment**  
   Upload the firmware located in the `arduino/` directory to the microcontrollers.

3. **Backend Server**  
   Set up the Flask–SocketIO server using the code in the `python/` directory. Dependencies are rather typical, therefore no requirements file is included here.

4. **AI Integration**  
   Configure access to Google's Gemini API to enable AI-driven analytics, specifically, the environment should contain the Gemini API keys.

## License

This project is licensed under the MIT License. 

---

For further information or collaboration inquiries, please contact the repository maintainer.
