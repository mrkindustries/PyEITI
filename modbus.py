from crc_16 import crc16

import time
import array
import serial
import struct 

#This laptop address in the MODBUS bus
PC_STATION = 4

#enumerate MODBUS protocol functions
FUNCTION_WRITE_SINGLE_REGISTER = 6
FUNCTION_READ_INPUT_REGISTER = 4

# Internal memory, jusy like a PLC
memory = [0] * 25000

#EITI stuff
PLC_REG_RTD = 0x0540
PLC_REG_TCPL = 0x0541
PLC_REG_CLP_RX = 0x0542
PLC_REG_CLP_TX = 0x0543
PLC_REG_EITI_STATUS = 0x0544
PLC_REG_BOARD_TEMP = 0x0545


class RTU_packet:
	def __init__ (self, packet_str):
		self.packet = array.array('B')
		self.packet.fromstring(packet_str)
		
		self.station  = self.packet[0]
		self.function = self.packet[1]
		self.register = self.packet[3] + (self.packet[2]<<8)		
		self.data     = self.packet[5] + (self.packet[4]<<8)
		if self.packet[4] & 0x80:	#its negative, perform 2's complement'
			self.data =  -(~self.data + (2<<15))
		self.CRC      = self.packet[6] + (self.packet[7]<<8)
		
		payload = struct.pack('BBBBBB',
		self.packet[0],
		self.packet[1],
		self.packet[2],
		self.packet[3],
		self.packet[4],
		self.packet[5])
		
		self.calculated_CRC = crc16(payload)
		
	def isvalid (self):
		if self.CRC == self.calculated_CRC:
			return True
		else:
			print "Bad CRC: \tGot: ", self.CRC, "Calculated:", self.calculated_CRC
			print "trying to sync again"
			UART.read(1)    
			return False

	def buildpacket(self):
		self.packet[0] = self.station = PC_STATION
		self.packet[1] = self.function
		if self.function == FUNCTION_READ_INPUT_REGISTER:
			self.packet[2] = 1
			self.packet[3] = self.data >> 8
			self.packet[4] = self.data & 0x00FF
			payload = struct.pack('BBBBB',
			self.packet[0],
			self.packet[1],
			self.packet[2],
			self.packet[3],
			self.packet[4])

		if self.function == FUNCTION_WRITE_SINGLE_REGISTER:
			self.packet[2] = self.register >> 8
			self.packet[3] = self.register & 0x00FF
#			self.packet[4] = self.data >> 8
			self.packet[5] = self.data & 0x00FF
		
			payload = struct.pack('BBBBBB',
			self.packet[0],
			self.packet[1],
			self.packet[2],
			self.packet[3],
			self.packet[4],
			self.packet[5])
		
		self.CRC = crc16(payload)
		
		if self.function == FUNCTION_READ_INPUT_REGISTER:
			self.packet[6] = self.CRC >> 8
			self.packet[5] = self.CRC & 0x00FF
		if self.function == FUNCTION_WRITE_SINGLE_REGISTER:
			self.packet[7] = self.CRC >> 8
			self.packet[6] = self.CRC & 0x00FF

	def tostruct(self):
	  
		packet_struct = struct.pack('BBBBBBBB',
		self.packet[0],
		self.packet[1],
		self.packet[2],
		self.packet[3],
		self.packet[4],
		self.packet[5],
		self.packet[6],
		self.packet[7])
		return packet_struct






# configure the serial connections (the parameters differ on the device you are connecting to)
UART = serial.Serial()
UART.baudrate = 9600
UART.port = '/dev/ttyUSB0'
UART.parity = 'O'      #ODD parity

UART.open()
UART.isOpen()

key_pressed = True
SHOW_ERROR_CODES = False
counter = 0

memory[PLC_REG_CLP_TX] = 20000 #here you can set the TX current in uAmperes

while key_pressed:  
	packet = RTU_packet( UART.read(8) )
	
	if packet.isvalid():
		if packet.function == FUNCTION_WRITE_SINGLE_REGISTER:
			memory[packet.register] = packet.data
			packet.buildpacket()
			UART.write(packet.tostruct() )
				 
		if packet.function == FUNCTION_READ_INPUT_REGISTER:
			packet.data = memory[packet.register]  #here you can set the TX current in uAmperes
			packet.buildpacket()
			UART.write(packet.tostruct() )

	print "EITI firmware  (rev %d)" % ((memory[PLC_REG_EITI_STATUS] & 0xFF00) >> 8)
	if (memory[PLC_REG_EITI_STATUS] & 0x0003):
		print "RTD temperature:   N/A C"
	else:
		print "RTD temperature:   %.1f C" % (float( memory[PLC_REG_RTD]) / 10)
	if (memory[PLC_REG_EITI_STATUS] & 0x000C):
		print "TCPL temperature:  N/A C"
	else:
		print "TCPL temperature:  %.1f C" % (float( memory[PLC_REG_TCPL]) / 10)
	print "Board temperature: %.1f C" % (float( memory[PLC_REG_BOARD_TEMP]) / 10)
	print "Current loop receiver:    %.2f mA" % (float( memory[PLC_REG_CLP_RX]) / 1000)
	print "Current loop transmitter: %.2f mA" % (float( memory[PLC_REG_CLP_TX]) / 1000)
	print "12v power supply: ", "ENABLED"
	
	if SHOW_ERROR_CODES:
		if (memory[PLC_REG_EITI_STATUS] & 0x0001):
			print "Warning: RTD open circuit"
		if (memory[PLC_REG_EITI_STATUS] & 0x0002):
			print "ERROR: RTD short circuit to GND"
		if (memory[PLC_REG_EITI_STATUS] & 0x0004):
			print "Warning: TCPL open circuit"
		if (memory[PLC_REG_EITI_STATUS] & 0x0008):
			print "ERROR: TCPL short circuit to GND or VDD"
		if (memory[PLC_REG_EITI_STATUS] & 0x00010):
			print "Warning: 4-20mA loop is transmititing less than 4mA"
		if (memory[PLC_REG_EITI_STATUS] & 0x00020):
			print "Warning: 4-20mA loop is transmititing more than 20mA"
		if (memory[PLC_REG_EITI_STATUS] & 0x00040):
			print "Warning: 4-20mA loop is receiving less than 4mA"
		if (memory[PLC_REG_EITI_STATUS] & 0x00080):
			print "Warning: 4-20mA loop is receiving more than 20mA"
	
	print "\n\n"
	
	if counter == 300:
		print 'Iterated 300 times, now exiting gracefully'
		break
	#counter = counter + 1

UART.close()
