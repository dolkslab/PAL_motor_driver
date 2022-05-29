port = /dev/ttyUSB0
filename = motor_driver.ino
comms_test: ${filename}
	arduino-cli compile -b arduino:avr:nano ${filename}
	arduino-cli upload -p ${port} -b arduino:avr:nano ${filename}
	
	
