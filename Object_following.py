# -*- coding: utf-8 -*-
"""
Created on Mon May 25 20:59:08 2020

@author: prajwal mj
"""

import cv2
import numpy as np
import matplotlib.pyplot as plt
import argparse
import imutils
#import RPi.GPIO as io
#simport asyncio
import speech_recognition as sr
from threading import Thread
#loop = asyncio.get_event_loop()
import time
cap=cv2.VideoCapture(0)

class Queue:
    def __init__(self):
        self.items = []

    def isEmpty(self):
        return self.items == []

    def enqueue(self, item):
        self.items.insert(0,item)

    def dequeue(self):
        return self.items.pop()

    def size(self):
        return len(self.items)
	
class Stack: 
	def __init__(self):   
         self.items = []

	def isEmpty(self):
         return self.items == []

	def push(self, item):
         self.items.append(item)

	def pop(self):
         return self.items.pop()

	def peek(self):
         return self.items[len(self.items)-1]

	def size(self):
         return len(self.items)	

	def clear(self):
		 self.items=[]
		 return self.items
		

			
			
global stk
stk=Stack()	

'''io.setmode(io.BCM)
d1=4
d2=22
d3=26
d4=12

io.setup(d1,io.OUT)

io.setup(d2,io.OUT)

io.setup(d3,io.OUT)

io.setup(d4,io.OUT)'''

def left(cx,cy):
	while(cx>0 and  cx<213 and  cy>=0 and  cy<160):
        #left forward
		'''io.output(d1,io.LOW)
		io.output(d2,io.LOW)
		io.output(d3,io.HIGH)
		io.output(d4,io.HIGH)'''
		print(" forward smooth left")
		cx,cy=get_frame(1)
	while(cx>0 and  cx<213 and  cy>=320 and  cy<480):
        #left reverse
		'''io.output(d1,io.LOW)
		io.output(d2,io.HIGH)
		io.output(d3,io.LOW)
		io.output(d4,io.HIGH)'''
		print("reverse smooth left")
		cx,cy=get_frame(1)
	while(cx>0 and  cx<=213 and  cy>=160 and  cy<320):
        #hrd left
		'''io.output(d1,io.LOW)
		io.output(d2,io.HIGH)
		io.output(d3,io.LOW)
		io.output(d4,io.LOW)'''
		print("hard left")
		cx,cy=get_frame(1)

def right(cx,cy):
    while(cx>=426 and  cx<640 and  cy>=0 and  cy<160):
        #right forward
        '''io.output(d1,io.LOW)
        io.output(d2,io.HIGH)
        io.output(d3,io.HIGH)
        io.output(d4,io.LOW)'''
        print("forward smooth right")
        cx,cy=get_frame(1)
    while(cx>=426 and  cx<640 and  cy>=320 and  cy<480):
        #right reverse
        '''io.output(d1,io.HIGH)
        io.output(d2,io.LOW)
        io.output(d3,io.LOW)
        io.output(d4,io.LOW)'''
        print("reverse smooth right")
        cx,cy=get_frame(1)
    while(cx>426 and  cx<=640 and  cy>=160 and  cy<320):
        #hard right
        '''io.output(d1,io.LOW)
        io.output(d2,io.HIGH)
        io.output(d3,io.HIGH)
        io.output(d4,io.HIGH)'''
        print("hard right")
        cx,cy=get_frame(1)

def forward(cx,cy):
		while((cx>=213 and  cx<426 and  cy>=0 and  cy<=160)or( cx==2000 and  cy==2000)):
			#forward
			print(cx,cy)
			'''io.output(d1,io.LOW)
			io.output(d2,io.LOW)
			io.output(d3,io.LOW)
			io.output(d4,io.LOW)'''
			if(cx==2000 and cy==2000):
				if(stk.isEmpty() is not True):
					if(stk.items[len(stk.items)-1]=='right' or stk.items[len(stk.items)-1]=='Right' or stk.items[len(stk.items)-1]=='left' or stk.items[len(stk.items)-1]=='Left' or stk.items[len(stk.items)-1]=='Stop' or stk.items[len(stk.items)-1]=='Reverse' or stk.items[len(stk.items)-1]=='Rivers' or stk.items[len(stk.items)-1]=='stop' or stk.items[len(stk.items)-1]=='reverse' or stk.items[len(stk.items)-1]=='rivers'):
						break
			print("forward")
			cx,cy=get_frame(0)	
        
def reverse(cx,cy):
		while((cx>=213 and  cx<426 and  cy>=320 and  cy<=480) or (cx==2000 and  cy==2000)):
			#reverse
			print(cx,cy)
			'''io.output(d1,io.LOW)
			io.output(d2,io.LOW)
			io.output(d3,io.HIGH)
			io.output(d4,io.LOW)'''
			if(cx==2000 and cy==2000):
				if(stk.isEmpty() is not True):
					if(stk.items[len(stk.items)-1]=='right' or stk.items[len(stk.items)-1]=='Right' or stk.items[len(stk.items)-1]=='left' or stk.items[len(stk.items)-1]=='Left' or stk.items[len(stk.items)-1]=='Stop' or stk.items[len(stk.items)-1]=='Forward' or stk.items[len(stk.items)-1]=='stop' or stk.items[len(stk.items)-1]=='forward'):
						break
			print("reverse")
			cx,cy=get_frame(0)

def stop(cx,cy):
		while((cx>=213 and  cx<426 and  cy>=160 and  cy<=320) or (cx==1000 and  cy==1000)):
			  #stop
			print(cx,cy)
			'''io.output(d1,io.LOW)
			io.output(d2,io.LOW)
			io.output(d3,io.LOW)
			io.output(d4,io.HIGH)'''
			if(cx==1000 and cy==1000):
				if(stk.isEmpty() is not True):
					if(stk.items[len(stk.items)-1]=='right' or stk.items[len(stk.items)-1]=='Right' or stk.items[len(stk.items)-1]=='left' or stk.items[len(stk.items)-1]=='Left' or stk.items[len(stk.items)-1]=='Forward' or stk.items[len(stk.items)-1]=='Reverse' or stk.items[len(stk.items)-1]=='Rivers' or stk.items[len(stk.items)-1]=='forward' or stk.items[len(stk.items)-1]=='reverse' or stk.items[len(stk.items)-1]=='rivers'):
						break
			print("stop")
			cx,cy=get_frame(1)

def s_right(cx,cy):
                while(cx==1000 and cy==1000):
                    print(cx,cy)
                    '''io.output(d1,io.LOW)
                    io.output(d2,io.HIGH)
                    io.output(d3,io.HIGH)
                    io.output(d4,io.HIGH)'''
                    if(stk.isEmpty() is not True):
					if(stk.items[len(stk.items)-1]=='stop' or stk.items[len(stk.items)-1]=='Stop' or stk.items[len(stk.items)-1]=='left' or stk.items[len(stk.items)-1]=='Left' or stk.items[len(stk.items)-1]=='Forward' or stk.items[len(stk.items)-1]=='Reverse' or stk.items[len(stk.items)-1]=='Rivers' or stk.items[len(stk.items)-1]=='forward' or stk.items[len(stk.items)-1]=='reverse' or stk.items[len(stk.items)-1]=='rivers'):
						break
                    print("right")
                    cx,cy=get_frame(1)

def s_left(cx,cy):
                while(cx==1000 and cy==1000):
                    print(cx,cy)
                    '''io.output(d1,io.LOW)
                    io.output(d2,io.HIGH)
                    io.output(d3,io.HIGH)
                    io.output(d4,io.HIGH)'''
                    if(stk.isEmpty() is not True):
					if(stk.items[len(stk.items)-1]=='right' or stk.items[len(stk.items)-1]=='Right' or stk.items[len(stk.items)-1]=='stop' or stk.items[len(stk.items)-1]=='Stop' or stk.items[len(stk.items)-1]=='Forward' or stk.items[len(stk.items)-1]=='Reverse' or stk.items[len(stk.items)-1]=='Rivers' or stk.items[len(stk.items)-1]=='forward' or stk.items[len(stk.items)-1]=='reverse' or stk.items[len(stk.items)-1]=='rivers'):
						break
                    print("left")
                    cx,cy=get_frame(1)
                    
          

	
        
def get_frame(status):
 while(True):
  ret,frame=cap.read()
  if(ret):
        hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        y_low=np.array([28,153,51])
        y_hi=np.array([36,255,255])
        mask=cv2.inRange(hsv,y_low,y_hi)
        res=cv2.bitwise_and(hsv,frame,mask=mask)
        kernel=np.ones((1,1),np.uint8)
        img_e=cv2.erode(mask,kernel,iterations=5)
        op1=cv2.morphologyEx(img_e,cv2.MORPH_OPEN,kernel)
        med1=cv2.medianBlur(op1,15)
        img=cv2.flip(med1,1)
        img2=cv2.flip(frame,1)
        #cv2.imshow('med1',med1)
        cv2.imshow('frame',frame)
	
        if(status==1):
            return get_centreup(med1)
        if(status==0):
            return get_centre(med1)


def get_centreup(fram):
    cnts = cv2.findContours(fram.copy(), cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]
    try:
            c = max(cnts, key = cv2.contourArea)
            M = cv2.moments(c)
            a = int(M["m10"] / M["m00"])
            b = int(M["m01"] / M["m00"])
    except:
            return(1000,1000)
    return a,b

def get_centre(fram):
    cnts = cv2.findContours(fram.copy(), cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]
    try:
            c = max(cnts, key = cv2.contourArea)
            M = cv2.moments(c)
            a = int(M["m10"] / M["m00"])
            b = int(M["m01"] / M["m00"])
    except:
            return(2000,2000)
    return a,b

def callback(recognizer, audio):
    try:
            yield(recognizer.recognize_google(audio))
    except:
            pass
	
def get_speech():
		#await asyncio.sleep(0.1)
	while(True):	
		r = sr.Recognizer()
		m = sr.Microphone()

		with m as source:
			#r.adjust_for_ambient_noise(source)
			#print("Say something!")
			audio = r.listen(source)

			
		'''stop_listening = r.listen_in_background(m, callback)
		time.sleep(3)'''
		try:
			audio2=r.recognize_google(audio)
			stk.push(audio2)
		except:
			pass
		'''audio=stop_listening()
		stk.push(audio)'''
		print(stk.items)
		#if (audio=='Forward' or audio=='forward' or audio=='reverse' or audio=='Reverse' or audio=='stop' or audio=='stop'):
			
thread1=Thread(target = get_speech)
thread1.start()	
	
#thread2=Thread(target=cam_read())
#thread2.start()	
	
while(True):
			#get_speech(3)
			#loop.call_later(3, get_speech, 3)
			#await asyncio.sleep(0)
			#if(que.items[0]=='Stop' or que.items[0]=='Reverse' or que.items[0]=='Rivers')
			cx,cy=get_frame(0)
			print(cx,cy)
			if(cx>=213 and  cx<426 and  cy>=0 and  cy<=160):
				stk.clear()
				while(True):
					cx,cy=get_frame(0)
					if(cx>=213 and  cx<426 and  cy>=0 and  cy<=160):
						forward(cx,cy)
					elif(cx==2000 and cy==2000):
						if(stk.isEmpty() is not True):
							if(stk.items[len(stk.items)-1]=='Stop' or stk.items[len(stk.items)-1]=='Reverse' or stk.items[len(stk.items)-1]=='Rivers' or stk.items[len(stk.items)-1]=='stop' or stk.items[len(stk.items)-1]=='reverse' or stk.items[len(stk.items)-1]=='rivers'):
								break
						forward(cx,cy)
					else:
						break
					
			elif(cx>=213 and cx<426 and cy>=160 and cy<=320):
				stk.clear()
				while(True):
					cx,cy=get_frame(1)
					if(cx>=213 and cx<426 and cy>=160 and cy<=320):
						stop(cx,cy)
					elif(cx==1000 and cy==1000):
						if(stk.isEmpty() is not True):
							if(stk.items[len(stk.items)-1]=='Forward' or stk.items[len(stk.items)-1]=='Reverse' or stk.items[len(stk.items)-1]=='Rivers' or stk.items[len(stk.items)-1]=='forward' or stk.items[len(stk.items)-1]=='reverse' or stk.items[len(stk.items)-1]=='rivers'):
								break
						stop(cx,cy)
					else:
						break
					
			elif(cx>=213 and cx<426 and cy>=320 and cy<=480):
				while(True):
					cx,cy=get_frame(0)
					if(cx>=213 and cx<426 and cy>=320 and cy<=480):
						reverse(cx,cy)
					elif(cx==2000 and cy==2000):
						if(stk.isEmpty() is not True):
							if(stk.items[len(stk.items)-1]=='Stop' or stk.items[len(stk.items)-1]=='Forward' or stk.items[len(stk.items)-1]=='forwatd' or stk.items[len(stk.items)-1]=='stop' or stk.items[len(stk.items)-1]=='reverse' or stk.items[len(stk.items)-1]=='rivers'):
								break
						reverse(cx,cy)
					else:
						break
					
			elif(cx>=0 and cx<213 and cy>=0 and cy<=160):
				stk.clear()
				while(True):
					cx,cy=get_frame(1)
					if(cx>=0 and cx<213 and cy>=0 and cy<=160):
						left(cx,cy)
					elif(cx==1000 and cy==1000):
						stop(cx,cy)
					else:
						break
					
			elif(cx>=0 and cx<213 and cy>=160 and cy<=320):
				while(True):
					cx,cy=get_frame(1)
					if(cx>=0 and cx<213 and cy>=160 and cy<=320):
						left(cx,cy)
					elif(cx==1000 and cy==1000):
						stop(cx,cy)
					else:
						break

			elif(cx>=0 and cx<213 and cy>=320 and cy<=480):
				stk.clear()
				while(True):
					cx,cy=get_frame(1)
					if(cx>=0 and cx<213 and cy>=320 and cy<=480):
						left(cx,cy)
					elif(cx==1000 and cy==1000):
						stop(cx,cy)
					else:
						break

			elif(cx>=426 and cx<640 and cy>=0 and cy<=160):
				stk.clear()
				while(True):
					cx,cy=get_frame(1)
					if(cx>=426 and cx<640 and cy>=0 and cy<=160):
						right(cx,cy)
					elif(cx==1000 and cy==1000):
						stop(cx,cy)
					else:
						break

			elif(cx>=426 and cx<640 and cy>=160 and cy<=320):
				stk.clear()
				while(True):
					cx,cy=get_frame(1)
					if(cx>=426 and cx<640 and cy>=160 and cy<=320):
						right(cx,cy)
					elif(cx==1000 and cy==1000):
						stop(cx,cy)
					else:
						break

			elif(cx>=426 and cx<640 and cy>=320 and cy<=640):
				stk.clear()
				while(True):
					cx,cy=get_frame(1)
					if(cx>=426 and cx<640 and cy>=320 and cy<=640):
						right(cx,cy)
					elif(cx==1000 and cy==1000):
						stop(cx,cy)
					else:
						break
			else:
				if(stk.isEmpty() is not True):
					#print(stk.items)
					if(stk.items[len(stk.items)-1]=='Forward' or stk.items[len(stk.items)-1]=='forward'):
						#print(stk.items)
						stk.clear()
						forward(2000,2000)
					elif(stk.items[len(stk.items)-1]=='Reverse' or stk.items[len(stk.items)-1]=='Rivers' or stk.items[len(stk.items)-1]=='reverse' or stk.items[len(stk.items)-1]=='rivers'):
						reverse(2000,2000)
						stk.clear()
					elif(stk.items[len(stk.items)-1]=='Stop' or stk.items[len(stk.items)-1]=='stop'):
						stop(1000,1000)
						stk.clear()
					elif(stk.items[len(stk.items)-1]=='Right' or stk.items[len(stk.items)-1]=='right'):
						s_right(1000,1000)
						stk.clear()
					elif(stk.items[len(stk.items)-1]=='Left' or stk.items[len(stk.items)-1]=='left'):
						s_left(1000,1000)
						stk.clear()
					
				else:
					print('empty stack')
					stop(cx,cy)
			k=cv2.waitKey(5) & 0xFF
			if(k==27):
				break



