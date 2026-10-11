Smart Bracelet for Epileptic Seizure Detection

An IoT wearable device developed to detect epileptic seizures in real-time 
and provide immediate assistance alerts. Powered by an Arduino Nano 33 BLE Sense, 
the bracelet simultaneously monitors heart rate anomalies and convulsive movements. 
Upon detecting a seizure, it triggers local visual and audio alarms to instruct bystanders,
while simultaneously transmitting a Bluetooth alert to a web interface for caretakers.

Features
Dual Sensor Detection: Combines the Arduino's built-in IMU (Accelerometer) for detecting 
convulsive movements with a MAX30100 sensor for abnormal heart rate (tachycardia/bradycardia)
detection.   Multi-Tiered Alert System: Features local alerts via a 5mm red LED and a DFPlayer 
Mini MP3 module that plays specific verbal instructions for bystanders. Remote alerts ("PULS", 
"CONVULSII", or "ALERTA") are sent via Bluetooth Low Energy (BLE) to a connected web interface.  
Escalating Audio Protocol: Plays an initial protective warning. If the alert is not manually dismissed 
via the physical push button within 3.5 minutes, the system escalates by requesting bystanders
to call emergency services (112).   False Alarm Prevention: Implements a 2-minute verification timer
requiring both an abnormal pulse and convulsive movements to be confirmed simultaneously, preventing
isolated sensor errors or old readings from triggering a full alarm. 

More details can be found in the pdf from the repository. 
