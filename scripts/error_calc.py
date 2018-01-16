#!/usr/bin/env python

"""
Copyright (c) 2017, Lars Niklasson
Copyright (c) 2017, Filip Slottner Seholm
Copyright (c) 2017, Fanny Sandblom
Copyright (c) 2017, Kevin Hoogendijk
Copyright (c) 2017, Nils Andren
Copyright (c) 2017, Alicia Gil Martin
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Chalmers University of Technology nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""


from math import *
import spline as s
from auto_master import *
from geometry import *

class ErrorCalc:
    def __init__(self):

        self.path = []
        self.line = (None, None)
        self.finish = False
        
        
    def appendPath(self, path):
        if path == []:
            return
            
        pos = [(p.x, p.y) for p in path]
        
        
        if pos[-1] == (-1,-1):
            pos.pop()
            self.finish = True
            
        
        #last 5 points, used for better splining
        l5 = [(p.x, p.y) for p in self.path[-5::]]
        
        #use linepoints for splining
        useLine = False
        if len(l5) <= 4:
            print "using linepoints for splining"
            (a,b) = self.line
            if a != None and b != None:
                l5 = [(b.x, b.y)] + l5
                useLine = True
            else:
                if b == None and a != None:
                    l5 = [(a.x, a.y)] + l5
                    useLine = True
                
        
        spl_pos = s.spline(l5 + pos)
        
        #remove first point so it's not duplicated
        if useLine:
            spl_pos = spl_pos[1::]
        
        
        #extend path so truck doesn't stop early due to lookahead
        if self.finish:
            print "finish appended"
            nl, last = spl_pos[-2:]
            d = getDirection(nl, last)
            la = getLookAheadPoint(last, d, GOAL_LOOKAHEAD)
            spl_pos.append(la)
        
        #remove points used for splining to make room for result
        self.path = self.path[:-5:]
        
        
        for (x,y) in spl_pos:
            self.path.append(Point(x, y))
        
        
        #add points to line if empty
        l1, l2 = self.line
        pathLength = len(self.path)
        
        if l1 != None and l2 == None:
            
            if pathLength >= 1:
                self.line = (l1, self.path.pop(0))
        
        elif self.line == (None,None):
            
            if pathLength == 1:
                self.line = (self.path.pop(0), l2)
            
            elif pathLength >= 2:
                self.line = (self.path.pop(0), self.path.pop(0))
                
       



        
        
    def reset(self):
        self.path = []
        self.line = (None, None)
        self.finish = False
    
    def reworkPath(self, path):
        self.reset()
        self.appendPath(path)
    
    def getPath(self):
        r = []
        l1,l2 = self.line
        if l1 != None:
            r.append(l1)
        if l2 != None:
            r.append(l2)
            
        if self.finish:
            p = self.path[:-1]
        else:
            p = list(self.path)
        return r + p

    def calculateError(self, (x, y)):
        p = Point(x,y)
        
        l1, l2 = self.line
        
        if l2 == None:
            return (0,0)
            
            
        while hasPassedLine(p, (l1, l2)):
            if len(self.path) == 0:
                self.line = (l2, None)
                return (0,0)
            
            tl1 = l2    
            tl2 = self.path.pop(0)
            self.line= (tl1, tl2)
            l1, l2 = self.line

        
        value = getDistanceFromLine(p, self.line)
        
        
        if isLeftOfLine(p, self.line):
            
            #+1 to account for points in self.line
            return (-value, len(self.path) + 1)
            
        else:
            return (value, len(self.path) + 1)

