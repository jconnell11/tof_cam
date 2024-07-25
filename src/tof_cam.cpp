// tof_cam.cpp : library for A010 Time-of-Flight camera interface
//
// Written by Jonathan H. Connell, jconnell@alum.mit.edu
//
///////////////////////////////////////////////////////////////////////////
//
// Copyright 2024 Etaoin Systems
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// 
///////////////////////////////////////////////////////////////////////////

#include <jhcTofCam.h>


///////////////////////////////////////////////////////////////////////////
//                          Global Variables                             //
///////////////////////////////////////////////////////////////////////////

//= Class with interface driver and post-processing cleanup.

static jhcTofCam tof;


///////////////////////////////////////////////////////////////////////////
//                           Main Functions                              //
///////////////////////////////////////////////////////////////////////////

//= Open connection to sensor and start background acquisition thread.
// returns 1 if okay, 0 or negative for error

extern "C" int tof_start ()
{
  return tof.Start();
}


//= Get a pointer to the most recent 16 bit depth image from sensor.
// buffer is always 100 x 100 with 16 bit pixels and 0.25mm resolution 
// with USB on left: scans right-to-left, top-down from upper right corner
// typically 14.8 fps, image guaranteed unchanged until next Range() call
// returns pixel buffer pointer, NULL if not ready or stream broken

extern "C" unsigned char *tof_range (int block)
{
  return tof.Range(block);
}


//= Stop background thread and close USB connection.

extern "C" void tof_done ()
{
  tof.Done();
}


/////////////////////////////////////////////////////////////////////////////
//                          Debugging Functions                            //
/////////////////////////////////////////////////////////////////////////////

//= Current range step (in mm) used by hardware sensor.

extern "C" int tof_step () 
{
  return tof.Step();
}


//= Get current raw sensor image for debugging.
// Note: image used by processing - do not alter pixels!

extern "C" unsigned char *tof_sensor () 
{
  return tof.Sensor();
}


//= Get current median filtered image for debugging.
// spatial filtering removes edge artifacts and shot noise
// Note: image used by processing - do not alter pixels!

extern "C" unsigned char *tof_median ()
{
  return tof.Median();
}


//= Get current Kalman filtered image for debugging.
// temporal filtering removes flickering and waves
// Note: image used by processing - do not alter pixels!

extern "C" unsigned char *tof_kalman () 
{
  return tof.Kalman();
}


//= Make an 8 bit grayscale image where close things are BRIGHTER.
// "sh" sets max range: 0 = 25cm, 1 = 51cm, 2 = 102cm, 3 = 204cm, 4 = 409cm 
// Note: must call Range(1) first to update source image to converter

extern "C" unsigned char *tof_night (int sh)
{
  return tof.Night(sh);
}

