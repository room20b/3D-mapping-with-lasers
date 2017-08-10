import time
import serial

ser = serial.Serial()
#ser.port = '/dev/tty.ForceT-DevB'
ser.port = '/dev/tty.HC-06-DevB'
#ser.port = "/dev/ttyS2"
ser.baudrate = 9600
ser.bytesize = serial.EIGHTBITS #number of bits per bytes
ser.parity = serial.PARITY_NONE #set parity check: no parity
ser.stopbits = serial.STOPBITS_ONE #number of stop bits
#ser.timeout = None          #block read
ser.timeout = 1            #non-block read
#ser.timeout = 2              #timeout block read
ser.xonxoff = False     #disable software flow control
ser.rtscts = False     #disable hardware (RTS/CTS) flow control
ser.dsrdtr = False       #disable hardware (DSR/DTR) flow control
ser.writeTimeout = 2     #timeout for write

ser.open()

if ser.isOpen():
	ser.flushInput()
	ser.flushOutput()
	ser.write(b'1\r')
	print('sent')
	time.sleep(0.5)
	a = time.time()
	while(ser.inWaiting()==0):
		pass
	print(ser.read())
	print(time.time()-a)
	a = time.time()
	while(ser.inWaiting()==0):
		pass
	print(ser.read())
	print(time.time()-a)
	a = time.time()
	while(ser.inWaiting()==0):
		pass
	print(ser.read())
	print(time.time()-a)
	ser.close()