#!/usr/bin/env python3
# encoding: utf-8

# =========================================================================
#
# tof_cam.py : Python wrapper for A010 Time-of-Flight camera interface
#
# Written by Jonathan H. Connell, jconnell@alum.mit.edu
#
# =========================================================================
#
# Copyright 2024-2025 Etaoin Systems
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# 
# =========================================================================

import numpy as np, cv2, sys
from ctypes import CDLL, POINTER, cast, c_ubyte, c_void_p

# serial port number (only matters for Windows)
port = 5

# bind shared library  
if sys.platform == 'win32':
  lib = CDLL('lib/tof_cam.dll')                  # Windows
else:
  lib = CDLL('lib/libtof_cam.so')                # Linux

# define return types of image functions (buffer pointers)
lib.tof_range.restype  = c_void_p
lib.tof_sensor.restype = c_void_p
lib.tof_median.restype = c_void_p
lib.tof_kalman.restype = c_void_p
lib.tof_night.restype  = c_void_p


# Python wrapper for A010 Time-of-Flight camera interface

class TofCam:

  # connect to Time-of-Flight sensor over USB
  # returns 1 if okay, 0 or negative for problem

  def Start(self):
    return lib.tof_start(port)


  # get 16 bit range image, possibly waiting for new frame (block = 1)
  # image is 100x100 pixels with depth in 0.25mm steps
  # image fmt: 0 = buffer pointer (int), 1 = OpenCV Mat (numpy ndarray)
  # returns pointer to image or None if not ready or broken

  def Range(self, block =0, fmt =1):
    return self.fmt_pels(lib.tof_range(block), fmt, 16)


  # convert a pointer to a byte sequence into an image object

  def fmt_pels(self, ptr, fmt, bits =8):
    if not ptr:
      return None
    if fmt <= 0:
      return ptr             # int = memory address
    if bits == 16:
      buf = cast(ptr, POINTER(c_ubyte * 20000))
      img = np.frombuffer(buf.contents, np.uint16)
    else:
      buf = cast(ptr, POINTER(c_ubyte * 10000))
      img = np.frombuffer(buf.contents, np.uint8)
    img.shape = (100, 100, 1)
    return img


  # cleanly disconnect imaging depth sensor

  def Done(self):
    lib.tof_done()


  # -------------------------------------------------------------------------

  # current range step (in mm) used by hardware sensor

  def Step(self):
    return lib.tof_step()


  # get current raw sensor image for debugging
  # image fmt: 0 = buffer pointer (int), 1 = OpenCV Mat (numpy ndarray)

  def Sensor(self, fmt =1):
    return self.fmt_pels(lib.tof_sensor(), fmt)


  # get current median filtered image for debugging
  # spatial filtering removes edge artifacts and shot noise
  # image fmt: 0 = buffer pointer (int), 1 = OpenCV Mat (numpy ndarray) 

  def Median(self, fmt =1):
    return self.fmt_pels(lib.tof_median(), fmt)


  # get current Kalman filtered image for debugging
  # temporal filtering removes flickering and waves
  # image fmt: 0 = buffer pointer (int), 1 = OpenCV Mat (numpy ndarray)

  def Kalman(self, fmt =1):
    return self.fmt_pels(lib.tof_kalman(), fmt)


  # get inverted 8 bit version depth image where bright means close 
  # max range from shift: 0 = 25cm, 1 = 50cm, 2 = 1m, 3 = 2m, 4 = 4m 
  # image fmt: 0 = buffer pointer (int), 1 = OpenCV Mat (numpy ndarray)
  # Note: must call Range(1) first to update source image conversion

  def Night(self, shift =1, fmt =1):
    return self.fmt_pels(lib.tof_night(shift), fmt)


# =========================================================================

# simple test program

if __name__ == "__main__":             
  sh = 1                               # default down-shift
  if len(sys.argv) > 1:
    if sys.argv[1].isdigit():
      sh = int(sys.argv[1])
    else:
      print("argument = depth down-shift (0-4: default = 1")

  # connect to sensor and make display window
  tof = TofCam()  
  if tof.Start() <= 0:
    print("Could not connect to TOF sensor!")
    sys.exit(0)
  cv2.namedWindow("Night")
  cv2.moveWindow("Night", 10, 10)

  # receive frames and display them
  print("Streaming images (%d cm max) ..." % (25 << sh))
  try:
    while tof.Range(1) is not None:
      img = tof.Night(sh)
      rot = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
      big = cv2.resize(rot, (300, 300), interpolation=cv2.INTER_NEAREST) 
      cv2.imshow("Night", big) 
      cv2.waitKey(1)                   # pump update message
  except KeyboardInterrupt:
    print("")

  # shutdown after error or Ctrl-C
  print("Sensor stopped")
  tof.Done()
