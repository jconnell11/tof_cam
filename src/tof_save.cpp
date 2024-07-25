// tof_save.cpp : uses jhcTofCam class image acquisition to record images
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

#include <jhcTofCam.h>


//= Writes 16 bit value, takes care of big-endian/little-endian problems.
// this version assumes stores values in little-endian fashion

void write16 (FILE *out, unsigned short val) 
{
  putc(val & 0xFF, out);
  putc((val >> 8) & 0xFF, out);
}


//= Writes 32 bit value, takes care of big-endian/little-endian problems.
// this version assumes stores values in little-endian fashion

void write32 (FILE *out, unsigned long val) 
{
  write16(out, (unsigned short)(val & 0xFFFF));
  write16(out, (unsigned short)((val >> 16) & 0xFFFF));
}


//= Save a 100x100 8 bit BMP format image.

void save_raw (const char *fname, const unsigned char *buf)
{
  FILE *out;
  int i;

  // sanity check then try opening file
  if ((buf == NULL) || (fname == NULL) || (*fname == '\0'))
    return;
  if ((out = fopen(fname, "wb")) == NULL)
    return;

  // write the file header (14 bytes)
  putc('B', out);                    // first 2 bytes are ascii "BM"
  putc('M', out);
  write32(out, 11078);               // whole file size 
  write32(out, 0);                   // Reserved 1 + 2
  write32(out, 54);                  // combined size of both headers

  // write the bitmapinfo header (40 bytes)
  write32(out, 40);                  // size of this header
  write32(out, 100);                 // image width
  write32(out, 100);                 // image height (bottom up)
  write16(out, 1);                   // number of planes
  write16(out, 8);                   // bits per pixel 
  write32(out, 0);                   // compression format (none)
  write32(out, 0);                   // image size (0 for uncompressed)
  write32(out, 0);                   // X pels/meter
  write32(out, 0);                   // Y pels/meter
  write32(out, 0);                   // colors used (0 can mean max)
  write32(out, 0);                   // important colors (all)

  // write fake color table for monochrome images (0:B:G:R for each index)
  for (i = 0; i < 256; i++)
  {
    putc(i, out);
    putc(i, out);
    putc(i, out);
    putc(0, out);                    // top byte of little-endian value
  }

  // save pixels and close file
  fwrite(buf, 1, 10000, out);
  fclose(out);
}


//= Get a 64 bit integer with number of nanoseconds since epoch.

long long time_stamp () 
{
  timespec ts;

  clock_gettime(CLOCK_BOOTTIME, &ts);
  return((long long)(ts.tv_sec * 1000000000 + ts.tv_nsec));
}


//= Number of milliseconds since reference timestamp.
// also updates timestamp provided to be current time

float ms_diff (long long& ref) 
{
  long long last = ref;

  ref = time_stamp();
  if (last <= 0)
    return 0.0;
  return((float)(0.000001 * (ref - last)));
}


///////////////////////////////////////////////////////////////////////////

//= Start capture and save first few images.
// defaults to 20 but can supply different number on command line

int main (int argc, char *argv[])
{
  char fname[80];
  jhcTofCam tof;
  long long start = 0;
  float loop, sum = 0.0;
  int i, n = 20; 

  // determine number of frames to save
  if (argc > 1)
    if (sscanf(argv[1], "%d", &n) != 1)
      return printf("usage: tof_save number-of-frames (10, 100, etc.)\n");
  system("rm -r raw");
  system("mkdir raw");

  // try to start up sensor
  if (tof.Start() <= 0)
  {
    printf("Could not connect to TOF sensor!\n");
    return -1;
  }

  // grab a bunch of frames
  printf("Streaming images ...\n");
  for (i = 0; i < n; i++)
  {
    loop = ms_diff(start);
    if (i >= 2)
      sum += loop;
    if (tof.Range(1) == NULL) 
      break;
    sprintf(fname, "raw/tof_%d_%dmm.bmp", i, tof.Step());
    save_raw(fname, tof.Sensor()); 
    if (i >= 2)
      printf("  [%d] %3.1f ms loop -> %4.2f fps\n", i, loop, 1000.0 / loop);
  }

  // success (at least partial)  
  tof.Done();
  printf("Saved %d images @ %4.2f fps (avg)\n", i, 1000.0 * (i - 2) / sum);
  return 0;
}

