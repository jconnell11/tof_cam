// tof_cam_win.h : library for A010 Time-of-Flight camera interface
//
// Written by Jonathan H. Connell, jconnell@alum.mit.edu
//
///////////////////////////////////////////////////////////////////////////
//
// Copyright 2024-2025 Etaoin Systems
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

#pragma once


// function declarations and link to library stub

#ifdef TOFCAM_EXPORTS
  #define DEXP __declspec(dllexport)
#else
  #define DEXP __declspec(dllimport)
  #pragma comment(lib, "lib/tof_cam.lib")
#endif


///////////////////////////////////////////////////////////////////////////
//                           Main Functions                              //
///////////////////////////////////////////////////////////////////////////

//= Open connection to sensor and start background acquisition thread.
// "port" is lower COMx number in Windows Device Manager (Linux ignores)
// returns 1 if okay, 0 or negative for error

extern "C" DEXP int tof_start (int port =3);


//= Get a pointer to the most recent 16 bit depth image from sensor.
// buffer is always 100 x 100 with 16 bit pixels and 0.25mm resolution 
// with USB on left: scans right-to-left, top-down from upper right corner
// typically 14.8 fps, image guaranteed unchanged until next Range() call
// returns pixel buffer pointer, NULL if not ready or stream broken

extern "C" DEXP const unsigned char *tof_range (int block =0);


//= Stop background thread and close USB connection.

extern "C" DEXP void tof_done ();


/////////////////////////////////////////////////////////////////////////////
//                          Debugging Functions                            //
/////////////////////////////////////////////////////////////////////////////

//= Current range step (in mm) used by hardware sensor.

extern "C" DEXP int tof_step (); 


//= Get current raw sensor image for debugging.
// Note: image used by processing - do not alter pixels!

extern "C" DEXP const unsigned char *tof_sensor ();


//= Get current median filtered image for debugging.
// spatial filtering removes edge artifacts and shot noise
// Note: image used by processing - do not alter pixels!

extern "C" DEXP const unsigned char *tof_median ();


//= Get current Kalman filtered image for debugging.
// temporal filtering removes flickering and waves
// Note: image used by processing - do not alter pixels!

extern "C" DEXP const unsigned char *tof_kalman ();


//= Make an 8 bit grayscale image where close things are BRIGHTER.
// "sh" sets max range: 0 = 25cm, 1 = 51cm, 2 = 102cm, 3 = 204cm, 4 = 409cm 
// Note: must call Range(1) first to update source image to converter

extern "C" DEXP const unsigned char *tof_night (int sh =1);

