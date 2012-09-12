from crc_16 import crc16
from matplotlib.pylab import *
#matplotlib.use('TkAgg')

from Tkinter import *
from time import sleep
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
from matplotlib.pyplot import *

import time
import array
import serial
import struct
import Tkinter as Tk
#import matplotlib.pyplot as plt
import threading
import csv


#This laptop address in the MODBUS bus
PC_STATION = 4

#enumerate MODBUS protocol functions
FUNCTION_WRITE_SINGLE_REGISTER = 6
FUNCTION_READ_INPUT_REGISTER = 4

# Internal memory, jusy like a PLC
memory = [0] * 25000
i = 1
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


class TemperatureControllerClass(threading.Thread):
	def exit(self):
		self.key_pressed = False

        def run(self):
                #---------------------------------------------------------------------------# 
                # start the temperature monitoring
                #---------------------------------------------------------------------------# 
                print "Temperature controller started\n"

		# configure the serial connections (the parameters differ on the device you are connecting to)
		UART = serial.Serial()
		UART.baudrate = 9600
		UART.port = '/dev/ttyUSB0'
		UART.parity = 'O'      #ODD parity

		UART.open()
		UART.isOpen()

		self.key_pressed = True
		SHOW_ERROR_CODES = False
		counter = 0

		memory[PLC_REG_CLP_TX] = 20000 #here you can set the TX current in uAmperes

		while self.key_pressed:  
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

#			print "\n\n"
			#print "EITI firmware  (rev %d)" % ((memory[PLC_REG_EITI_STATUS] & 0xFF00) >> 8)
			#if (memory[PLC_REG_EITI_STATUS] & 0x0003):
				#print "RTD temperature:   N/A C"
			#else:
				#print "RTD temperature:   %.1f C" % (float( memory[PLC_REG_RTD]) / 10)
			#if (memory[PLC_REG_EITI_STATUS] & 0x000C):
				#print "TCPL temperature:  N/A C"
			#else:
				#print "TCPL temperature:  %.1f C" % (float( memory[PLC_REG_TCPL]) / 10)
			#print "Board temperature: %.1f C" % (float( memory[PLC_REG_BOARD_TEMP]) / 10)
			#print "Current loop receiver:    %.2f mA" % (float( memory[PLC_REG_CLP_RX]) / 1000)
			#print "Current loop transmitter: %.2f mA" % (float( memory[PLC_REG_CLP_TX]) / 1000)
			#print "12v power supply: ", "ENABLED"
			
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
			
			
			if counter == 300:
				print 'Iterated 300 times, now exiting gracefully'
				exit()
				break
			#counter = counter + 1

		UART.close()

 


def destroy(e): sys.exit()


t = TemperatureControllerClass()
t.start()


root = Tk.Tk()
root.wm_title("EITI")
root.minsize(300,300)
root.geometry("700x500")
root.geometry("+300+100")

TX_current = StringVar()
TX_current.set("4.0")
L1 = Tk.Label(root, text="TX current")
L1.pack( side = Tk.RIGHT)
TX_current_entry = Tk.Entry(root, textvariable = TX_current, background = 'white')
TX_current_entry.pack(side = Tk.RIGHT)

TCPL_label = StringVar()
TCPL_label.set("N/A")

TCPL_label_frame = Label( root, textvariable=TCPL_label)
TCPL_label_frame.pack(side = Tk.RIGHT)

RTD_label = StringVar()
RTD_label.set("N/A")

RTD_label_frame = Label( root, textvariable=RTD_label)
RTD_label_frame.pack(side = Tk.RIGHT)


f = Figure(figsize=(30,30), dpi=80)
temperature_plt = f.add_subplot(211)
canvas = FigureCanvasTkAgg(f, master=root)
canvas.show()
canvas.get_tk_widget().pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)


table = ['RTD temperature',
	  'TCPL temperature',
	  '4-20mA RX',
	  '4-20mA TX',
	  'EITI Status']

writer = csv.writer(open('temp_acq.csv', 'wb'), delimiter=',', quotechar='"', quoting=csv.QUOTE_NONNUMERIC)
writer.writerow(table)

#prepare plot
ion()                     # turn off interactive mode
x = arange(0,300,0.5)     # x axis will go from 0 to 300
RTD_temperature  = 0 * x
TCPL_temperature = 0 * x
current_loop_RX  = 0 * x
current_loop_TX  = 0 * x

f.patch.set_facecolor('white')
temperature_plt.set_title('EITI interfaces')
temperature_plt.set_xlabel('Time [sec]')
temperature_plt.set_ylabel('Temperature [C]')
RTD_line,TCPL_line, = temperature_plt.plot(x, RTD_temperature, 'r,', x, TCPL_temperature, 'b,')	#the comma is the linestyle
temperature_plt.legend( (RTD_line,TCPL_line), ("RTD","TCPL") )          
RTD_line.axes.set_ylim(-50,50)
RTD_line.set_antialiased(True)
TCPL_line.set_antialiased(True)


current_plot = f.add_subplot(212)
current_plot.set_xlabel('Time [sec]')
current_plot.set_ylabel('Current [mA]')
RX_line,TX_line, = current_plot.plot(x, current_loop_RX, 'r,', x, current_loop_TX, 'b,')
current_plot.legend( (RX_line,TX_line), ("RX","TX") )          
RX_line.axes.set_ylim(-1,24)
RX_line.set_antialiased(True)
TX_line.set_antialiased(True)



def update_plots():
	global i
	if (memory[PLC_REG_EITI_STATUS] & 0x0003):
		RTD_temperature[i]  = RTD_temperature[i-1]
		RTD_label.set("N/A")
	else:
		RTD_temperature[i]  = float( memory[PLC_REG_RTD]) / 10
		RTD_label.set(RTD_temperature[i])

	if (memory[PLC_REG_EITI_STATUS] & 0x000C):
		TCPL_temperature[i] = TCPL_temperature[i-1]
		TCPL_label.set("N/A")
	else:
		TCPL_temperature[i] = float( memory[PLC_REG_TCPL]) / 10
		TCPL_label.set(TCPL_temperature[i])
		
	memory[PLC_REG_CLP_TX] = int(float(TX_current.get()) * 1000)
	current_loop_RX[i]  = float( memory[PLC_REG_CLP_RX]) / 1000
	current_loop_TX[i]  = float( memory[PLC_REG_CLP_TX]) / 1000
	EITI_status         = 40

	# Plot the data
	RTD_line.set_ydata(RTD_temperature)
	TCPL_line.set_ydata(TCPL_temperature)
	RX_line.set_ydata(current_loop_RX)
	TX_line.set_ydata(current_loop_TX)
	f.canvas.draw()
	
	table = [ RTD_temperature[i], TCPL_temperature[i], current_loop_RX[i], current_loop_TX[i], EITI_status]
	writer.writerow(table)
	i = i +1
	root.after(500, update_plots)


root.after(500, update_plots)
root.mainloop()

t.exit()
exit()
