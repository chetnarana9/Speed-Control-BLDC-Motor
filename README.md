# Speed-Control-BLDC-Motor
Overview:

This project demonstrates PWM-based speed control of a Brushless DC (BLDC) motor using NodeMCU ESP8266 and an Electronic Speed Controller (ESC). The system enables smooth motor operation with real-time speed monitoring using Hall sensors and IoT-based control.

Key Features:
1. PWM-based speed control via NodeMCU ESP8266
2. ESC-driven electronic commutation of BLDC motor
3. Hall sensor–based RPM measurement
4. Real-time speed display on 16×2 LCD
5. IoT monitoring using Blynk platform

Working Principle: NodeMCU generates a PWM signal (1–2 ms, 50 Hz). ESC interprets PWM and drives the BLDC motor. Hall sensors measure motor speed (RPM). Speed data is displayed on LCD and can be monitored remotely.

Components Used: NodeMCU ESP8266, BLDC Motor (1000 KV), Electronic Speed Controller (ESC), Hall-effect sensors, 16×2 LCD Display, Power supply, jumper wires
