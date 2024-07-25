// tof_show.cpp : uses jhcTofCam class to display intermediate images
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

#include <opencv2/opencv.hpp> 

#include <jhcTofCam.h>


//= Convert scan by columns top-down rightmost first to OpenCV ordering.
// also magnifies 100x100 image to 300x300 using block duplication

void mono_rot3 (unsigned char *dest, const unsigned char *src)
{
  int x, y, i, j;                      // src coords
  unsigned char *d, *d0 = dest + 297;  // end of top row
  const unsigned char *s = src;   

  if ((dest == NULL) || (src == NULL))
    return;
  for (x = 99; x >= 0; x--, d0 -= 3)
    for (y = 0, d = d0; y < 100; y++, s++)
      for (j = 0; j < 3; j++, d += 297)
        for (i = 0; i < 3; i++, d++)
          *d = *s;
}


///////////////////////////////////////////////////////////////////////////

//= Continuously show sensor and processed images until Ctrl-C.
// can alter depth down-shifting using command line (defaults to 1)

int main (int argc, char *argv[])
{
  jhcTofCam tof;
  cv::Mat raw, med, kal, nite;
  const unsigned char *pels;
  int shift = 1;

  // determine 16 bit depth value shift
  if (argc > 1)
    if (sscanf(argv[1], "%d", &shift) != 1)
      return printf("usage: tof_show depth-downshift (0, 3, etc.)\n");

  // try to start up sensor
  if (tof.Start() <= 0)
  {
    printf("Could not connect to TOF sensor!\n");
    return -1;
  }

  // make display images
  raw  = cv::Mat(300, 300, CV_8UC1);
  med  = cv::Mat(300, 300, CV_8UC1);
  kal  = cv::Mat(300, 300, CV_8UC1);
  nite = cv::Mat(300, 300, CV_8UC1);

  // make display windows
  cv::namedWindow("Sensor");
  cv::namedWindow("Median");
  cv::namedWindow("Kalman");
  cv::namedWindow("Night");

  // arrange in 2x2 grid
  cv::moveWindow("Sensor",  10,  10);
  cv::moveWindow("Median", 320,  10);
  cv::moveWindow("Kalman", 320, 350);
  cv::moveWindow("Night",   10, 350);

  // keep grabbing frames from sensor
  printf("Streaming images (%d cm max) ...\n", 25 << shift);
  while ((pels = tof.Range(1)) != NULL)
  {
    // load OpenCV image data 
    mono_rot3(raw.data, tof.Sensor());    
    mono_rot3(med.data, tof.Median());  
    mono_rot3(kal.data, tof.Kalman());  
    mono_rot3(nite.data, tof.Night(shift));  

    // display new frames
    cv::imshow("Sensor", raw);    
    cv::imshow("Median", med); 
    cv::imshow("Kalman", kal);  
    cv::imshow("Night", nite);       
    cv::waitKey(1);                    // pump update message
  }

  // only gets here on failure
  tof.Done();
  printf("Sensor stopped\n");
  return 0;
}

