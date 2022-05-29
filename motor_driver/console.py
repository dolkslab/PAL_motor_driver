import serial
import time
def int_byte(x):
	return bytes(chr(x), 'utf-8')


con_attempts = 10

with serial.Serial('/dev/ttyUSB0', 9600, timeout=0.1, write_timeout = 0.02) as ser:


	ser.reset_input_buffer()
	ser.reset_output_buffer()
  
	while True:
		a = input("input command: ")
		if a[0] == "q":
			ser.write(bytes([0, 2]))
			break
		elif a[0] == "p":
			ser.write(bytes([0, 1]))
			while ser.in_waiting < 1:
				time.sleep(0.001)
			if ser.read(ser.in_waiting) == bytes([1,0]):
				print("pong")
			
		elif a[0] == "s":
			ser.write(bytes([0, 2]))
			while ser.in_waiting < 1:
				time.sleep(0.001)
			ser.read(ser.in_waiting) 
			
		elif a[0] == "c":
			current = min(abs(int(a[4:])), 255)
			mot_sel = int(a[2] == "R")*255
			mot_dir = int(int(a[4:]) > 0)*255
			
			ser.write(bytes([3, 3, mot_sel, mot_dir, current]))
			print([3, 3, mot_sel, mot_dir, current])
			while ser.in_waiting < 1:
				time.sleep(0.001)
			print(ser.read() == bytes([1]))
			print(int.from_bytes(ser.read(), "little"))
			
		elif a[0] == "v":
			vel = max(-(2**15), min(int(a[2:]), 2^15-1))
			ser.write(bytes([2, 4]))
			ser.write(vel.to_bytes(2,"little"))
			print(vel.to_bytes(2,"little"))
			while ser.in_waiting < 1:
				time.sleep(0.001)
			print(ser.read() == bytes([1]))
			print(ser.read(ser.in_waiting))
							
		else:
			print("Available commands:")
			print("p: ping, wait for pong response")
			print("s: stop current to motors")
			print("c [0:255]: set motor current")
			print("v [{}:{}]: set motor velocity".format(-(2**15), 2**15-1))
			pass
		













