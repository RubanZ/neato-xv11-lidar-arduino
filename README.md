# neato-xv11-lidar-arduino
Protocol description for Arduino



A full revolution will yield 90 packets, containing 4 consecutive readings each. The length of a packet is 22 bytes. This amounts to a total of 360 readings (1 per degree) on 1980 bytes. Each packet is organized as follows:

