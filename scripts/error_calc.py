#!/usr/bin/env python

from math import *
import spline as s

class Point:
    def __init__(self, x , y):
        self.x = x
        self.y = y
        
    def __repr__(self):
        return "(" + str(self.x) + " " + str(self.y) + ")"


def getDirection((x1,y1),(x2,y2)):
    return atan2(y2-y1, x2-x1)

def getLookAheadPoint((x,y),direction, lookahead):
    #TODO checkwith right coordinate system
    return (x+lookahead*cos(direction), y+lookahead*sin(direction))
    
def getDistance(p0, p1, p2):
    return abs((p2.x - p1.x)*(p1.y-p0.y) - (p1.x-p0.x)*(p2.y-p1.y)) / (sqrt((p2.x-p1.x)*(p2.x-p1.x) + (p2.y-p1.y)*(p2.y-p1.y)))
    
def isLeft(p0,p1,p2):
    return ((p2.x - p1.x)*(p0.y - p1.y) - (p2.y - p1.y)*(p0.x - p1.x)) >0 #decides if the error is to the left of centerline or not
    
class ErrorCalc:
    def __init__(self):

        self.path = []
        self.line = (None, None)
        
    def appendPath(self, path):
        
        pos = [(p.x, p.y) for p in path]
        
        l5 = [(p.x, p.y) for p in self.path[-5::]]
        
        r = False
        if len(l5) <= 4:
            (a,b) = self.line
            if a != None and b != None:
                l5 = [(b.x, b.y)] + l5
                r = True
            else:
                if b == None and a != None:
                    l5 = [(a.x, a.y)] + l5
                    r = True
                
        
        qpos = s.spline(l5 + pos)
        
        if r:
            qpos = qpos[1::]
        
        self.path = self.path[:-5:]
        
        
        for (x,y) in qpos:
            self.path.append(Point(x, y))
        
        
        
        if self.line[0] != None and self.line[1] == None:
            if len(self.path) >= 1:
                self.line = (self.line[0], self.path.pop(0))
        elif self.line == (None,None):
            if len(self.path) == 1:
                self.line = (self.path.pop(0), self.line[1])
            elif len(self.path) >= 2:
                self.line = (self.path.pop(0), self.path.pop(0))
        
        print "self.line", self.line
        print "self.path", self.path
        
        
    def reset(self):
        self.path = []
        self.line = (None, None)
    
    def reworkPath(self, path):
        self.reset()
        self.appendPath(path)
        
    

    def calculateError(self, (x, y)):
        p0 = Point(x,y)
        
        (p1,p2) = self.line
        
        if p2 == None:
            return (0,0)
            
            
        while self.isAboveEnd(p1,p2,p0):
            if len(self.path) == 0:
                self.line = (p2,None)
                return (0,0)
            
            tempP1=p2    
            tempP2=self.path.pop(0)
            self.line= (tempP1,tempP2)
            (p1,p2) = self.line

        value = getDistance(p0,p1,p2)
        
        
        if(isLeft(p0,p1,p2)):
            return (value, len(self.path)+1)
        else:
            return (-value, len(self.path)+1)

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
