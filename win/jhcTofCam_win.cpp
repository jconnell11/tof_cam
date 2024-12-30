// jhcTofCam_win.cpp : interface to Sipeed MaixSense A010 Time-of-Flight sensor
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

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>

#include "jhcTofCam_win.h"


///////////////////////////////////////////////////////////////////////////
//                      Creation and Initialization                      //
///////////////////////////////////////////////////////////////////////////

//= Destructor cleans up files and any allocated items.

jhcTofCam::~jhcTofCam ()
{
  Done();
}


//= Default constructor initializes certain values.

jhcTofCam::jhcTofCam ()
{
  // 16 bit scaling for "unit" 
  build_lut();

  // strip header from packet
  raw = pkt + 16;

  // auto-ranging
  sat = 80;                            // max frac saturated
  pct = 50;                            // histogram percentile
  ihi = 150;                           // desired range span
  cx0 = 25;                            // central ROI
  cy0 = 25;
  cw  = 50;
  ch  = 50;

  // temporal smoothing
  f0 = 0.1f;                           // estimate time constant
  nv = 64.0;                           // expect 3 bits noise (8^2)
  vlim = 32;                           // too much flicker

  // procesing state
  ser = -1;
  ok = -1;
  run = 0;
  frame = 0;
}


//= Construct lookup table for converting 8 bit depth to 16 bit.
// first index is current "unit" - 1, entries are in 0.25mm steps

void jhcTofCam::build_lut ()
{
  int u, pel;

  for (u = 1; u <= 9; u++)
    for (pel = 0; pel < 256; pel++)
      norm[u - 1][pel] = (unsigned short)(4 * u * pel);  
}


///////////////////////////////////////////////////////////////////////////
//                              Main Functions                           //
///////////////////////////////////////////////////////////////////////////

//= Open connection to sensor and start background acquisition thread.
// "port" is lower COMx number in Windows Device Manager (Linux ignores)
// returns 1 if okay, 0 or negative for error

int jhcTofCam::Start (int port)
{
  // establish USB serial connection
  ok = -1;
  if (ser.SetSource(port, 115200) <= 0)
    return ok;

  // configure and start sensor
  ser.TxArray((UC8 *)"AT+DISP=3\r", 10);         // needs live display!
  Sleep(50);                                     // 50ms min between commands
  ser.TxArray((UC8 *)"AT+UNIT=2\r", 10);         // 2mm depth step                 
  unit = 2;
  pend = 2;   

  // initialize rotating buffers
  fill = d0;
  done = NULL;
  lock = NULL;
  fresh = -2;                          // first 2 are stale                       

  // launch receiver and pre-processor thread
  run = 1;
  pthread_create(&hoover, NULL, absorb, (void *) this);
  ok = 1;
  return ok;
}


//= Get a pointer to the most recent 16 bit depth image from sensor.
// buffer is always 100 x 100 with 16 bit pixels and 0.25mm resolution 
// with USB on left: scans right-to-left, top-down from upper right corner
// typically 14.8 fps, image guaranteed unchanged until next Range() call
// returns pixel buffer pointer, NULL if not ready or stream broken

const unsigned char *jhcTofCam::Range (int block)
{
  int wait = 0;

  // check if source is operational and new frame is ready
  if (ok <= 0)  
    return NULL;
  while (fresh <= 0)
  {
    if (block <= 0)                    // return immediately
      return NULL;
    if (wait++ > 500)                  // barf after 0.5 sec
      return NULL;
    Sleep(1);                          // 1 ms loop
  }

  // swap buffers to be sure output pointer remains valid
  pthread_mutex_lock(&data);
  lock = done;                         // mark as in-use
  fresh = 0;
  pthread_mutex_unlock(&data);
  return lock;
}


//= Stop background thread and close USB connection.

void jhcTofCam::Done ()
{
  // stop receiver and pre-processor thread
  if (run > 0)
  {
    run = 0;                                
    pthread_join(hoover, NULL);   
  }

  // stop transmitter and release serial port
  if (ser.Valid() > 0)
  {
    ser.TxArray((UC8 *)"AT+UNIT=0\r", 10);       // stretched depth
    Sleep(50);
    ser.TxArray((UC8 *)"AT+DISP=1\r", 10);
    ser.Close();
  }

  // mark as un-initialized
  ok = -1;
}


///////////////////////////////////////////////////////////////////////////
//                        Background Acquisition                         //
///////////////////////////////////////////////////////////////////////////

//= Background thread continually receives serial bytes into raw buffers.
// also automatically adjusts range resolution and filters pixels

pthread_ret jhcTofCam::absorb (void *tof)
{
  jhcTofCam *me = (jhcTofCam *) tof;

  while (me->run > 0) 
  {
    // get sensor pixels
    if (me->sync() <= 0)
    {
      printf(">>> jhcTofCam: Frame sync timeout!\n");
      break;
    }
    if (me->fill_raw() <= 0)
    {
      printf(">>> jhcTofCam: Image pixels timeout!\n");
      break;
    }
    
    // analyze and filter image
    me->auto_range();
    me->median5x5();
    me->flywheel();
    me->reformat();
    me->swap_bufs();
  }
  me->ok = 0;                          // stream ended               
  return NULL;                            
}


//= Look for beginning of image packet = start code + correct length.
// returns 1 when found, 0 if stream broken

int jhcTofCam::sync () 
{
  unsigned char b;
  int start = 0;

  // find start of next packet
  while (1)
  {      
    // start code = 0x00 0xFF
    start++;
    if ((b = ser.Rcv()) < 0)
      return 0;
    if (b != 0x00)
      continue;
    if ((b = ser.Rcv()) < 0)
      return 0;
    if (b != 0xFF)
      continue;

    // packet length 10016 = 0x2720 (little-endian)
    if ((b = ser.Rcv()) < 0)
      return 0;
    if (b != 0x20)
      continue;
    if ((b = ser.Rcv()) < 0)
      return 0;
    if (b == 0x27)
      break;
  }

  // assume extra bytes are response to "unit" command
  if ((start > 1) && (frame > 2))
    depth_step();
  return 1;
}


//= Fills the "raw" image buffer with received serial bytes.
// whole packet = 16 byte header + 10000 byte image + 2 bytes at end
// returns 1 when successful, 0 if stream broken

int jhcTofCam::fill_raw ()
{
  int rc, n = 0;                      

  while (1)
  {
    rc = ser.RxArray(pkt + n, 10018 - n);   
    if (rc <= 0)
      return 0;                        // timeout
    n += rc;
    if (n >= 10018) 
      break;
    Sleep(18);                         // accumulate more bytes
  }
  return 1;
}


//= Mark filtering as completed and shuffle output images.

void jhcTofCam::swap_bufs ()
{
  pthread_mutex_lock(&data);
  done = fill;                         // most recent complete
  fresh += 1;
  if (fill == d0)
    fill = ((lock != d1) ? d1 : d2);
  else if (fill == d1)
    fill = ((lock != d0) ? d0 : d2);
  else                                 // lock == d2             
    fill = ((lock != d0) ? d0 : d1);
  pthread_mutex_unlock(&data);
  frame++;                             // increment frame count
}


///////////////////////////////////////////////////////////////////////////
//                            Image Filtering                            //
///////////////////////////////////////////////////////////////////////////

//= Median filter "raw" image with 5x5 mask to give "med" image.
// updates histogram by removing box back edge and adding front edge
// keeps track of lowest values in histogram to reduce scanning
// takes about 1ms on Pi 4 (3.5x faster than straightforward coding)

void jhcTofCam::median5x5 ()
{
  int x, y, i, j, now, nowx, pel, hi, sub, bot, v, cnt; 
  unsigned char *d = med;
  const unsigned char *s = raw;
 
  // apply 5x5 median filter over each row of image 
  for (y = 0; y < 10000; y += 100)
  {
    // set up histogram for x = 0 edge:  -2  -1   x  +1  +2
    //                           pixel:  s0  s0  s0  s1  s2        
    for (i = 0; i < 256; i++)                 
      vals[i] = 0;
    bot = 255;
    for (j = -200; j < 300; j += 100)            // 5 rows high
    {
      // copy top or bottom rows if needed
      now = y + j;
      now = ((now <= 0) ? 0 : ((now < 9900) ? now : 9900));

      // 3 left values (copies of s0) 
      pel = *(s + now);                          // left edge pixel
      bot = ((pel < bot) ? pel : bot);
      vals[pel] += 3;                    
  
      // 2 right values
      pel = *(s + now + 1);
      bot = ((pel < bot) ? pel : bot);
      vals[pel] += 1;
      pel = *(s + now + 2);
      bot = ((pel < bot) ? pel : bot);
      vals[pel] += 1;
    }

    // evaluate 5x5 patch at current position (histogram is up to date)
    for (x = 0; x < 100; x++, d++)
    {
      // find histogram bin number that n'th sorted value would fall into
      sub = 0;
      cnt = 0;                         // lowest[0] = bot
      for (hi = bot; hi < 256; hi++)   // scan up from bot
        if ((v = vals[hi]) > 0)                    
        {
          if (cnt < 6)                 // save possible bot replacements
            lowest[cnt++] = hi;
          sub += v;                  
          if (sub >= 13)               // find top filled bin
            break;
        }
      *d = (unsigned char) hi;         // median value
 
      // unless last pixel in row, shift histogram box to right 
      if (x >= 99)
        continue;                      // still increment d
      cnt = 0;                         // bot = lowest[0]

      // subtract off old column on left: [-2] -1   x  +1  +2  +3
      //                        new span:       *   *  x'   *   *        
      nowx = x - 2;
      nowx = ((nowx <= 0) ? 0 : nowx);
      for (j = -200; j < 300; j += 100)          // 5 rows high
      {
        now = y + j; 
        now = ((now <= 0) ? 0 : ((now < 9900) ? now : 9900));
        pel = *(s + now + nowx);
        if ((pel == bot) && (vals[pel] <= 1))    // replace bot
          bot = lowest[++cnt];         
        vals[pel] -= 1;
      }

      // add in next column on right: -2  -1   x  +1  +2 [+3]
      //                    new span:      *   *  x'   *   *        
      nowx = x + 3; 
      nowx = ((nowx < 99) ? nowx : 99);          // copy right edge if needed
      for (j = -200; j < 300; j += 100)          // 5 rows high
      {
        now = y + j; 
        now = ((now <= 0) ? 0 : ((now < 9900) ? now : 9900));
        pel = *(s + now + nowx);
        bot = ((pel < bot) ? pel : bot);
        vals[pel] += 1;
      }
    }
  }
}


//= Perform Kalman-like temporal filtering on "med" image.
// assumes true process is a random walk in intensity
// 
//   measurement:  M  = P + Vm       where Vm = variance in measurement
//       process:  P' = d * P + c    where c is expected jumpiness 
//                                     and d is a time decay constant 
//
// smoothed values are in "avg" and variance estimates in "var"

void jhcTofCam::flywheel ()
{
  int fi = (int)(256.0 * f0 + 0.5), cfi = 256 - fi;
  int i, diff, vm, k, val, mn = (int)(256.0 * nv + 0.5);
  unsigned char *p = avg, *v = var;
  const unsigned char *m = med;

  // if first frame then initialize average and variance
  if (frame <= 0)
  {
    memcpy(p, m, 10000);     // copy source
    memset(v, 0, 10000);     // no flickering
    return;
  }

  // project from last step and add in new measurement 
  for (i = 10000; i > 0; i--, m++, p++, v++)
  {
    // see how much raw pixels are varying to get mix factor
    diff = (*m) - (*p);
    vm   = cfi * (*v) + fi * diff * diff;
    k    = (vm << 8) / (vm + mn);

    // use mix factor to update estimates of average and variance
    val  = (((*p) << 8) + k * diff + 128) >> 8;
    *p   = ((val <= 0) ? 0 : ((val < 255) ? val : 255));
    val  = ((256 - k) * (vm >> 1) + 16384) >> 15;
    *v   = ((val <= 0) ? 0 : ((val < 255) ? val : 255));
  }
}


//= Mask unreliable pixels and convert to 16 bit in "done" image.
// ignore if bad sensor, bad average, or high variance

void jhcTofCam::reformat ()
{
  int i;
  const unsigned short *sc = norm[unit - 1];
  unsigned short *d = (unsigned short *) fill;
  const unsigned char *s = raw, *p = avg, *v = var;

  for (i = 0; i < 10000; i++, d++, s++, p++, v++)
    if ((*s >= 255) || (*p >= 255) || (*v > vlim)) 
      *d = 65535;
    else 
      *d = sc[*p];           // adjust for "unit" resolution
}


/////////////////////////////////////////////////////////////////////////////
//                            Range Adjustment                             //
/////////////////////////////////////////////////////////////////////////////

//= Histogram center of image to select better depth resolution.
// sets "pend" as the newly requested resolution for "unit"

void jhcTofCam::auto_range ()
{
  char cmd[20] = "AT+UNIT=2\r";
  int area = cw * ch, skip = 100 - cw;
  int x, y, miss, stop, bulk, goal, sum = 0;
  const unsigned char *s = raw + 100 * cy0 + cx0;

  // first few frames have bad data
  if (frame < 2)
    return;

  // histogram pixels in center of raw image
  for (bulk = 0; bulk < 256; bulk++)
    cent[bulk] = 0;
  for (y = 0; y < ch; y++, s += skip)
    for (x = 0; x < cw; x++, s++)
      cent[*s] += 1;

  // find fraction saturated and intensity for given percentile
  miss = (int)((100.0 * cent[255]) / area + 0.5);
  stop = (int)(0.01 * pct * (area - cent[255]) + 0.5);
  for (bulk = 0; bulk < 255; bulk++)
  {
    sum += cent[bulk];
    if (sum >= stop)
      break;
  }

  // change depth resolution to better span range
  goal = (int)(unit * bulk / (float) ihi + 0.5);    
  goal = ((goal <= 1) ? 1 : ((goal < 9) ? goal : 9));
  if ((miss > sat) && (goal <= unit) && (unit < 9))
    goal = unit + 1;

  // possibly request step size change
  if ((goal != unit) && (pend == unit))
  {
    pend = goal;
    cmd[8] = '0' + pend;
    ser.TxArray((UC8 *) cmd, 10);                // needs confirmation
  }
}


//= Register that new "unit" of resolution is in effect.

void jhcTofCam::depth_step ()
{
  int sc[256];
  int i, f;
  unsigned char *p = avg, *v = var;

  // adjust temporal filter average values 
  f = (unit << 8) / pend;
  for (i = 0; i < 256; i++)
    sc[i] = (f * i + 128) >> 8;
  for (i = 10000; i > 0; i--, p++)
    *p = sc[*p];    

  // adjust temporal filter variance values 
  f = ((unit * unit) << 8) / (pend * pend);
  for (i = 0; i < 256; i++)
    sc[i] = (f * i + 128) >> 8;
  for (i = 10000; i > 0; i--, v++)
    *v = sc[*v];    

  // record current sensor resolution
  unit = pend;
}


/////////////////////////////////////////////////////////////////////////////
//                          Debugging Functions                            //
/////////////////////////////////////////////////////////////////////////////

//= Make an 8 bit grayscale image where close things are BRIGHTER.
// "sh" sets max range: 0 = 25cm, 1 = 51cm, 2 = 102cm, 3 = 204cm, 4 = 409cm 
// Note: must call Range(1) first to update source image to converter

const unsigned char *jhcTofCam::Night (int sh)
{
  int i, v, dn = sh + 2;
  unsigned char *d = nite;
  const unsigned short *s = (unsigned short *) lock;                

  if (s == NULL)                       // from Range(1)
    return NULL;
  for (i = 10000; i > 0; i--, s++, d++)
  {
    v = *s >> dn;
    v = ((v < 255) ? v : 255);
    *d = (unsigned char)(255 - v);
  }
  return nite;
}
