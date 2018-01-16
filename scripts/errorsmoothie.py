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
from error_calc import *
from auto_master import *
from threading import Thread
from threading import Timer
import rospy
from math import sqrt
import time
from geometry import *

class ErrorSmoothie:
    def __init__(self, automaster):
        self.am = automaster

        self.reset()


    def reset(self):
        self.current_camera = -1
        self.switch_camera_allowed = True

        self.one_tag_cc = False

        self.last_front_point_nc = None
        self.last_front_point_cc = None

        self.last_error = 0

        self.error_diff = 0

    def smoothErrorDiff(self):

        iterations = (SMOOTHING_TIME/SMOOTHING_DT)
        de = self.error_diff / iterations

        for i in range(int(iterations)):
            self.error_diff -= de
            time.sleep(SMOOTHING_DT)
        self.error_diff = 0

    def setSwitchCameraAllowed(self):
        print "switch camera allowed"
        self.switch_camera_allowed = True


    def gvPositionsHandler(self, data):
        p1 = (data.p1.x, data.p1.y)
        p2 = (data.p2.x, data.p2.y)
        tagid1 = data.tagid1
        tagid2 = data.tagid2
        cameraid = data.cameraid


        if self.current_camera == -1:
            self.current_camera = cameraid


        if self.current_camera == cameraid:
            # current camera

            if p2 == (0,0) and tagid2 == 0:
                #one tag

                if tagid1 == 1:
                    #only back tag
                    print "camera %s: only back tag" % cameraid
                    self.one_tag_cc = True

                else:
                    # only front tag
                    print "camera %s: only front tag" % cameraid


                    if self.last_front_point_cc == None:
                        print "camera %s: no last front point, use no lookahead" % cameraid
                        lookAheadPoint = p1
                        direction = None
                    elif getDistanceBetweenPoints(p1, self.last_front_point_cc) < ONLY_FRONT_TAG_TOO_CLOSE_DIST:
                        print "camera %s: last front point too close, use no lookahead" % cameraid
                        lookAheadPoint = p1
                        direction = None
                    else:
                        print "camera %s: getting direction from last front point, use small lookahead" % cameraid
                        #maybe not using any lookahead is better...?
                        direction = getDirection(self.last_front_point_cc, p1)
                        lookAheadPoint = getLookAheadPoint(p1, direction, ONLY_FRONT_TAG_LOOKAHEAD)

                    error, dist = self.am.error_calc.calculateError(lookAheadPoint)
                    self.last_error = error

                    error = error - self.error_diff


                    self.last_front_point_cc = p1
                    if direction != None:
                        self.am.updateLatest(getLookAheadPoint(p1, direction, 65-100), direction)

                    self.am.processError(error, dist)



            else:
                #two tags

                if tagid1 !=1:
                    tp = p1
                    p1 = p2
                    p2 = tp

                direction = getDirection(p1,p2)
                lookAheadPoint = getLookAheadPoint(p2, direction, LOOKAHEAD)

                error,dist = self.am.error_calc.calculateError(lookAheadPoint)
                self.last_error = error

                error = error - self.error_diff

                self.latest_front_point_cc = p2
                if direction != None:
                    self.am.updateLatest(getLookAheadPoint(p2, direction, 65-100), direction)

                self.am.processError(error, dist)

        else:
            if not self.switch_camera_allowed: #just switched cameras, old camera not interesting
                print "camera %s: cant switch camera yet bro" % cameraid
                return

            if p2 == (0,0) and tagid2 == 0:
                #one tag

                if tagid1 == 1:
                    #only back tag
                    print "camera %s: only back tag" % cameraid

                else:
                    #only front tag
                    print "camera %s: only front tag" % cameraid
                    if self.one_tag_cc:
                        print "camera %s: cc can only see one tag, switching camera to %s"  % (cameraid, cameraid)

                        if self.last_front_point_nc == None:
                            print "camera %s: no last front point, use no lookahead" % cameraid
                            lookAheadPoint = p1
                            direction = None
                        elif getDistanceBetweenPoints(p1, self.last_front_point_nc) < ONLY_FRONT_TAG_TOO_CLOSE_DIST:
                            print "camera %s: last front point too close, use no lookahead" % cameraid
                            lookAheadPoint = p1
                            direction = None
                        else:
                            print "camera %s: getting direction from last front point, use small lookahead" % cameraid
                            #maybe not using any lookahead is better...?
                            direction = getDirection(self.last_front_point_nc, p1)
                            lookAheadPoint = getLookAheadPoint(p1, direction, ONLY_FRONT_TAG_LOOKAHEAD)


                        if direction != None:
                            self.am.updateLatest(getLookAheadPoint(p1, direction, 65-100), direction)

                        error, dist = self.am.error_calc.calculateError(lookAheadPoint)

                        self.error_diff = error - self.last_error
                        print "camera %s: error_diff %f" % (cameraid, self.error_diff)
                        self.last_error = error

                        self.current_camera = cameraid

                        self.last_front_point_cc = p1
                        self.last_front_point_nc = None
                        self.one_tag_cc = False

                        self.switch_camera_allowed = False

                        print "camera %s: start smoothing" % cameraid
                        Thread(target = self.smoothErrorDiff).start()
                        Timer(SWITCH_CAMERA_COOLDOWN, self.setSwitchCameraAllowed).start()


                        error = error - self.error_diff


                        self.am.processError(error, dist)

                    else:
                        print "camera %s: cc can still see both tags" % cameraid
                        self.last_front_point_nc = p1


            else: #two tags


                print "camera %s: two tags" % cameraid
                print "camera %s: switching to camera %s" % (cameraid, cameraid)

                if tagid1 !=1:
                    tp = p1
                    p1 = p2
                    p2 = tp

                direction = getDirection(p1,p2)
                lookAheadPoint = getLookAheadPoint(p2, direction, LOOKAHEAD)

                error,dist = self.am.error_calc.calculateError(lookAheadPoint)



                self.error_diff = error - self.last_error

                print "camera %s: error_diff %f" % (cameraid, self.error_diff)

                self.current_camera = cameraid

                self.last_front_point_cc = p2
                self.last_fron_point_nc = None
                self.one_tag_cc = False

                self.switch_camera_allowed = False

                print "camera %s: start smoothing" % cameraid
                Thread(target = self.smoothErrorDiff).start()
                Timer(SWITCH_CAMERA_COOLDOWN, self.setSwitchCameraAllowed).start()
                if direction != None:
                    self.am.updateLatest(getLookAheadPoint(p2, direction, 65-100), direction)

                error = error - self.error_diff

                self.am.processError(error, dist)
