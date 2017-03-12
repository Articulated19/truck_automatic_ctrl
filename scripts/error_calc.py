#!/usr/bin/env python

from math import *
from Queue import Queue

class Point:
    def __init__(self, x , y):
        self.x = x
        self.y = y
        self.line = None


def getLookAheadPoint((backp0x, backp0y), (frontp0x, frontp0y), lookahead):
	prevV =(frontp0x - backp0x , frontp0y - backp0y)
	mag = sqrt((frontp0x - backp0x)**2 + (frontp0y - backp0y)**2)
	nPrevV = tuple( comp/mag for comp in prevV )
	(x,y) = tuple(comp*lookahead for comp in nPrevV) #the vector from prevToPoint to newPoint
	x= x+frontp0x
	y= y+frontp0y
	return (x,y)
	
def getLookAheadPoint((x,y),direction, lookahead):
	#TODO checkwith right coordinate system
	return (x+lookahead*cos(direction), y+lookahead*sin(direction))
	
def getDistance(p0, p1, p2):
	return abs((p2.x - p1.x)*(p1.y-p0.y) - (p1.x-p0.x)*(p2.y-p1.y)) / (sqrt((p2.x-p1.x)*(p2.x-p1.x) + (p2.y-p1.y)*(p2.y-p1.y)))
	
def isLeft(p0,p1,p2):
	return ((p2.x - p1.x)*(p0.y - p1.y) - (p2.y - p1.y)*(p0.x - p1.x)) >0 #decides if the error is to the left of centerline or not
	
class ErrorCalc:
    def __init__(self, newPath=None):

        self.queue = Queue()
        self.line = (None, None)
        
    def appendPath(self, path):
		for position in path:
	        self.queue.put(Point(position.x, position.y))
	    if self.line[0] != None and self.line[1] == None:
			if self.queue.qsize >= 1:
				self.line[1] = self.queue.get()
		elif self.line == (None,None):
			if self.queue.qsize == 1:
				self.line[0] = self.queue.get()
		    elif self.queue.qsize >= 2:
				self.line[0] = self.queue.get()
				self.line[1] = self.queue.get()
	    
	    
    def reset(self):
		self.queue = Queue()
		self.line = (None, None)
    
    def reworkPath(self, path):
		self.queue = Queue()
		appendPath(path)
    

    def calculateError(self, x, y):
        p0 = Point(x,y)
        
        (p1,p2) = self.line
        
        if p2 == None:
			return (-1,0)
			
			
        while self.isAboveEnd(p1,p2,p0):
            if self.queue.qsize() == 0:
				self.line = (p2,None)
				return (-1,0)
			
			tempP1=p2	
            tempP2=self.queue.get()
            self.line= (tempP1,tempP2)
            (p1,p2) = self.line

        value = getDistance(p0,p1,p2)
        
        if(isLeft(p0,p1,p2)):
            return (value, self.queue.qsize())
        else:
            return (-value, self.queue.qsize())

    def isAboveEnd (self,begin, end, p0):
        #checks if a point is passed the end point of a line.
        if begin.x - end.x !=0 and begin.y - end.y !=0:
            slope = float(begin.y - end.y) / float(begin.x - end.x)
            prependularSlope = (-1)/slope
            prependularM = end.y - end.x*prependularSlope
            if begin.y < end.y:
                #going up
                return (p0.x*prependularSlope + prependularM - p0.y) < 0
            else:
                #going down
                return (p0.x*prependularSlope + prependularM - p0.y) > 0
        elif begin.x - end.x:
            #going straight in x direction
            if begin.x < end.x:
                #going right
                return p0.x > end.x
            else:
                #going left
                return p0.x < end.x
        else:
            #going straight in y direction
            if begin.y < end.y:
                #going up
                return p0.y > end.y
            else:
                #going down
                return p0.y < end.y
