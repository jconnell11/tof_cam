// jhcTofCam.h : interface to Sipeed MaixSense A010 Time-of-Flight sensor
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

#include <pthread.h>


//= Interface to Sipeed MaixSense A010 Time-of-Flight sensor.
// rotates through 3 element image buffers: xfill, xdone, xlock
// uses faster median algorithm with partial histogram scans
// pipelines spatial and temporal filters for lower latency

class jhcTofCam
{
// PRIVATE MEMBER VARIABLES
private:
  // camera connection and health
  int ser, ok;  

  // background receiver and pre-processing
  pthread_t hoover;
  pthread_mutex_t data;
  int run;

  // sensor input image
  unsigned char pkt[10018];
  unsigned char *raw;

  // auto-ranging
  int cent[256];
  int unit, pend;

  // median filtering
  unsigned char med[10000];
  int vals[256], lowest[6];
 
  // temporal smoothing
  unsigned char avg[10000], var[10000];
  int frame;
 
  // resolution scaling
  unsigned short norm[9][256];

  // final 16 bit depth images
  unsigned char d0[20000], d1[20000], d2[20000];
  unsigned char *fill, *done, *lock;
  int fresh;

  // debugging 8 bit depth image
  unsigned char nite[10000];


// PUBLIC MEMBER VARIABLES
public:
  // auto-ranging parameters
  int sat, pct, ihi, cx0, cy0, cw, ch; 

  // temporal smoothing parameters
  float f0, nv;
  int vlim;


// PUBLIC MEMBER FUNCTIONS
public:
  // creation and initialization
  ~jhcTofCam ();
  jhcTofCam ();

  // main functions
  int Start (int port =0);
  const unsigned char *Range (int block =0);
  void Done ();

  // debugging functions (not sync'd with background)
  int Step () const {return unit;}
  const unsigned char *Sensor () const {return raw;}
  const unsigned char *Median () const {return((unsigned char *) med);}
  const unsigned char *Kalman () const {return((unsigned char *) avg);}
  const unsigned char *Night (int sh =0);


// PRIVATE MEMBER FUNCTIONS
private:
  // creation and initialization
  void build_lut ();

  // main functions
  int open_usb ();

  // background thread functions
  static void *absorb (void *tof);
  int sync ();
  int fill_raw ();
  void swap_bufs ();

  // image filtering
  void median5x5 ();
  void flywheel ();
  void reformat ();

  // range adjustment
  void auto_range ();
  void depth_step ();

};

