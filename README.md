This is from the 8th semester project with the theme being a system of systems.

The objective of this project was trying to create a noise measurement model by using a electret microphone to record signal noises. 
By including other sensors to confirm environmental conditions which could attenuate sound LoRa is used to send all the necessary data as a 
packet to a Rasbperry Pi, which in turn receives the packet and breaks it into topics which will be published with MQTT to Node-RED.

Data can be seen from the Node-RED dashboard and a worldmap to indicate the loudness of the area. Lastly all the data are locally saved to the 
Raspberry Pi in the form of a CSV file that includes a timestamp.
