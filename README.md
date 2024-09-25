this is a project I created for basic UAVs its a inertial-navigation system this project uses a raspberry pi, mpu6050, and a bmp338. The project was to create a open source minimalistic navigation system for UAV's 
and drones alike. The script that was supposed to utilize mass sensor fusion to gain positional awarness yet due to minimalism positional awarness is lacking and estimated awarness is what is avialable.
the code collects information from the sensors and uses a kalman filter to clean the data for better positional awarness. when combined with a nrf24l01 module you can send the black box data to another computer
allowing for you to see and estimate where the drone is.
