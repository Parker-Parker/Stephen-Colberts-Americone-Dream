import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import numpy as np
import serial
import sys

if len(sys.argv) == 1:
	port = 'COM12'
else:
	port = sys.argv[1]
n = 100000
ser = serial.Serial(port, 9600)

# Fixing random state for reproducibility
#plt.ion()

plt.rcdefaults()
fig, ax = plt.subplots()

# Example data
people = ('up/down','pitch', 'right/left', 'yaw', 'forward/back')
x_pos = np.arange(len(people))
bar_height = [1,1,1,1,1]
ax.bar(x_pos, bar_height, align='center', color='green', ecolor='black')
	
def setter():
	ax.set_xticks(x_pos)
	ax.set_xticklabels(people)
	#ax.invert_yaxis()  # labels read top-to-bottom
	ax.set_xlabel('axis')
	ax.set_ylabel('deviation')
	ax.set_title('Forces and Torques')
	plt.ylim((-200,200))

setter()

def runRead():
	linein = ser.readline().rstrip()
	if len(linein)>4:
		valid_utf8 = True
		try:
			linein.decode('utf-8')
		except UnicodeDecodeError:
			valid_utf8 = False
		if valid_utf8:
			inputs = linein.split(b',')
			inputs = [x for x in filter(None, inputs)]
			numeric = True
			for n in inputs :
				try:
					float(n)
				#if not n.strip(b'-').decode('utf-8').isnumeric() :
				except ValueError:
					numeric = False
			if numeric :
				bar_height = [ float(x) for x in inputs ]
				print((inputs,bar_height))#, inputs.decode('utf-8')))
				return (True, bar_height)
			else:
				print("not numeric!")
				print((inputs))#,bar_height))#, inputs.decode('utf-8')))
				return (False, [1,1,1,1,1])

def animate(i,bar_height,x_pos):
	#pullData = open("sampleText.txt","r").read()
	#dataArray = pullData.split('\n')
	(dod,bh) = runRead()
	if dod:
		bar_height = bh
		ax.clear()
		print(bar_height)
		ax.bar(x_pos, bar_height, align='center', color='green', ecolor='black')
		setter()
	
ani = animation.FuncAnimation(fig, animate, fargs=(bar_height,x_pos), interval=30)
plt.show()


	
