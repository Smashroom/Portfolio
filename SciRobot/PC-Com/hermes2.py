import bluetooth
import datetime
import unicodedata
import requests

import Xlib
import Xlib.display
import Xlib.threaded

from threading import Thread
from threading import Timer
import string

import json
from json import JSONEncoder
import psutil	
 
import matplotlib.pyplot as pyplot

sessionActivities = {}
sessionPidActivity={}
nameActivity=[]
timeActivity=[]
activity={}

disp = Xlib.display.Display()
root = disp.screen().root

dataFile=open('data2.txt','a')
flagstheThread=True

NET_WM_NAME = disp.intern_atom('_NET_WM_NAME')
NET_ACTIVE_WINDOW = disp.intern_atom('_NET_ACTIVE_WINDOW')
NET_ACTIVE_WINDOW_PID=disp.intern_atom('_NET_WM_PID')
root.change_attributes(event_mask=Xlib.X.FocusChangeMask)

class MyEncoder(JSONEncoder):
	def default(self,o):
		return o.__dict__
class Activity(object):
	def __init__(self,name,pid):
		object.__init__(self)
		self.name = name
		self.pid = pid
		self.counter=0
	def update(self):
		self.counter+=1
	def totalTime(self):
		return self.counter
	def nameofProcess(self):
		return self.name
	def __str__(self):
		return str(self.name) + " - " + str(self.pid) + " - " + str(self.counter)
		
class myThread(Thread):
	def __init__(self,threadID):
		Thread.__init__(self)
		self.threadID=threadID
	def run(self):
		global flagstheThread	
		if(self.threadID==1): 		#Listening Thread			
			print "Starting " + str(self.threadID)		
			ListenBluetooth()
			print"Exiting " + str(self.threadID)
		if(self.threadID==2):		#Reading and Writing Thread				
			print "Starting " + str(self.threadID)			
			ReadBluetooth()
			print"Exiting" + str(self.threadID)				
def ListenBluetooth():
	global dataFile
	global flagstheThread
	global sessionActivities
	global sessionPidActivity
	while True:
		data3=sock.recv(7)
		if "Start" in data3:
			dataFile=open('data2.txt','a')
			now=datetime.datetime.now()
			currentTime=now.strftime("%Y-%m-%d %H:%M:%S")
			dataFile.write(currentTime+' \n\n')
			dataFile.write('-------The Processes--------'+'\n\n')	
			#print "Buradayim button1"		
			flagstheThread=False
		if "Y" in data3:
			#print "Buradayim button2"
			flagstheThread=True
			writetoList()
			dataFile=open('data2.txt','a')		
def ReadBluetooth():
	global flagstheThread
	global sessionActivities
	global sessionPidActivity
	captureTitle()
def captureTitle():
	while True:
		if flagstheThread==True:
			continue
		else: 
			break
	#print "Burada miyim "
	try:
		Timer(2,captureTitle).start()
	except:
		#print "zamaan olmadi"
		pass	
	global sessionActivities
	global sessionPidActivity
	try:
		window_id = root.get_full_property(NET_ACTIVE_WINDOW, Xlib.X.AnyPropertyType).value[0]
		window = disp.create_resource_object('window', window_id)
		window.change_attributes(event_mask=Xlib.X.PropertyChangeMask)
		window_name = window.get_full_property(NET_WM_NAME, 0).value
		pids= window.get_full_property(NET_ACTIVE_WINDOW_PID,Xlib.X.AnyPropertyType).value
		window_name = window_name.replace(u"\u2014", "")
		window_name = window_name.replace(u"\xb7", "")	
		for i in pids:
			titleController=sessionActivities.get(window_name)			
			p=process_by_pid(i)
			pidController=sessionPidActivity.get(i)
			if pidController!=None and titleController!=None:
				sessionActivities[window_name].update()
				sessionPidActivity[i].update()
			else:		
				currentActivity=Activity(str(p.name()),str(i))
				sessionPidActivity[i] = currentActivity
				sessionActivities[window_name]=currentActivity
		event = disp.next_event()
	except:
		#print "kaydedemedi "
		pass	

def writetoList():
	global dataFile
	global sessionActivities
	global timeActivity
	global nameActivity
	global activity
	summary=0
	for i in sessionActivities:
		summary+=sessionActivities[i].totalTime()
		if sessionActivities[i].totalTime()!=0:
			if sessionActivities[i].nameofProcess() in activity.keys():	
				x=activity[sessionActivities[i].nameofProcess()]
				x=x+sessionActivities[i].totalTime()
				activity[sessionActivities[i].nameofProcess()]=x			
			else:
				activity[sessionActivities[i].nameofProcess()]=sessionActivities[i].totalTime()
	dataFile.write(str(summary)+'\n')
	print summary
	try:
		data=MyEncoder().encode(sessionActivities)	
		dataFile.write(data)
		print data
		now=datetime.datetime.now()
		finishTime=now.strftime('\n\n'+"%Y-%m-%d %H:%M:%S")
		dataFile.write(finishTime+'\n\n')
		dataFile.write('------The Processes Finished--------'+'\n\n')
		dataFile.close() 
	except:
		#print "yazdiramadi"
		pass
	try:
		sessionActivities={}
		pyplot.axis("equal")
		pyplot.pie(
			activity.values(),
			labels=activity.keys(),
			autopct="%1f%%"
		)
		pyplot.title("Processes")
		pyplot.show()
	except:
		pass
	
	

def process_by_pid(pid):
        return psutil.Process(int(pid))
"""
bd_address="20:14:04:15:21:12"
"""
bd_address_phone="----"
port=1

sock=bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sock.connect((bd_address,port))
print 'Connect'
sock.settimeout(None)
# sock.send("10")

#Create new threads
thread1=myThread(1)
thread2=myThread(2)

#Start new threads
thread1.start()
thread2.start()


#Join the micro threads to the main thread
thread1.join()
thread2.join()
#print "Exit bre"

sock.close()

