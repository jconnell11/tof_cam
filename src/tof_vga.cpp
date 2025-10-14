// tof_vga.cpp : uses jhcTofCam class to generate VGA-sized depth images
//
// Written by Jonathan H. Connell, jconnell@alum.mit.edu
//
///////////////////////////////////////////////////////////////////////////
//
// Copyright 2025 Etaoin Systems
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


// minimum and maximum

#define __min(a, b) (((a) < (b)) ? (a) : (b))
#define __max(a, b) (((a) > (b)) ? (a) : (b))


// ------------------------------ GLOBALS -------------------------------- 

// TOF cam resampling values

int sx[480], fx[480], sy[640], fy[640];


// image buffers in left-to-right bottom-up format

unsigned short vga[640 * 480 * 2];
cv::Mat gray(480, 640, CV_8UC1);


// ---------------------------- INTERPOLATION ---------------------------- 

//= Build arrays of sampling coordinate for each destination value.
// dest:   -5   -4   -3   -2   -1 |  0    1    2    3    4 |  5    6    7    8    9 |  A    B    C    D    E |  F
//    x:  -1.4 -1.2 -1.0 -0.8 -0.6|-0.4 -0.2  0.0  0.2  0.4| 0.6  0.8  1.0  1.2  1.4| 1.6  1.8  2.0  2.2  2.4| 2.6
//   ix:   -1   -1   -1   -1   -1 |  0'   0'   0    0    0 |  0    0    1    1    1 |  1    1    2    2    2 |  3
//  src:             -1           |            0           |            1           |            2           |
//   fx:                          | 0.0  0.0  0.0  0.2  0.4| 0.6  0.8  0.0  0.2  0.4| 0.6  0.8  0.0  0.0  0.0|
//                                |  *    *                |                        |                 *    * |
// image is 100x100 square pixels, 66.6 degs wide and high (flen = 76.1)
// set rf = 365.2 to fully fill VGA height (2 * atan(240/365.2) = 66.6 degs)
// originally from jhcSwapBody class

void tof_sampling ()
{
  double x, y, step = 100.0 / 480.0;             // 0.2083
  int dx, dy, ix, iy;

  // determine input sample position for each output row
  for (dy = 0; dy < 480; dy++)
  {
    x = 49.5 + step * (239.5 - dy);
    ix = (int) x;
    sx[dy] = ix;                                 // pixel offset
    if ((ix <= 0) || (ix >= 99))
      fx[dy] = 0;
    else
      fx[dy] = (int)(256.0 * (x - ix) + 0.5);    // mix next factor
  }

  // determine input sample line offset for each output column
  for (dx = 0; dx < 640; dx++)
  {
    y = 49.5 + step * (319.5 - dx);
    iy = (int) y;
    sy[dx] = 100 * iy;                           // line offset
    if ((iy <= 0) || (iy >= 99))
      fy[dx] = 0;
    else
      fy[dx] = (int)(256.0 * (y - iy) + 0.5);    // mix next factor
  }
}


//= Expand original 100x100 image to 640x480 with bilinear interpolation.
// needs valid sx[], fx[], sy[], and fy[] arrays from tof_sampling()
// pels are scanned right-to-left, top-down from upper right corner
// needs a square of 4 valid pixels to make mixed pixel (shrinks mask)
// will not interpolate across big horizontal or vertical depth jumps
// originally from jhcSwapBody class

void set_z16_tof (unsigned short *dest, const unsigned char *buf) 
{
  const unsigned short *s = (unsigned short *) buf;
  unsigned short *d = dest;
  int dx, dy, i, hf, ref, vf, sw, se, nw, ne, bot, top, pel;
  int hjump = (int)(4.0 * 101.6 + 0.5), vjump = (int)(4.0 * 101.6 + 0.5);  // inches

  // rescan and uniformly stretch input image
  for (dy = 0; dy < 480; dy++)
  {
    // make sure source column is valid
    i = sx[dy];
    if ((i < 0) || (i >= 100))
    {
      memset(d, 0xFF, 1280);                                   // skip whole line
      d += 640;                                                
      continue;
    }
    hf = fx[dy];                                               // horizontal mixing

    // interpolate source pixels to get destination line
    // NW:NE:SW:SE square refers to pels array (rotated 90 degs wrt dest)
    for (dx = 0; dx < 640; dx++, d++)
    {
      // check that reference address is within source
      *d = 0xFFFF;                                             // default
      ref = sy[dx] + i;
      if ((ref < 0) || (ref >= 10000))
        continue;
      vf = fy[dx];                                             // vertical mixing

      // mix lower quad pixels (SW & SE) but only if valid 
      sw = s[ref];
      if (sw == 0xFFFF)                                        // quad partially invalid
        continue;
      bot = sw;                                                // left of edge 
      if (hf > 0)
      {      
        se = s[ref + 1];
        if (se == 0xFFFF)                                      // quad partially invalid
          continue;
        if (abs(se - sw) < vjump)
          bot = ((sw << 8) + hf * (se - sw)) >> 8;             // full mix 
        else if (hf >= 128)
          bot = se;                                            // right of edge
      }

      // check if vertical mix needed
      if (vf <= 0)                                             
      {
        *d = (unsigned short) bot;
        continue;
      }

      // mix upper quad pixels (NW & NE) but only if valid 
      nw = s[ref + 100];
      if (nw == 0xFFFF)                                        // quad partially invalid
        continue;
      top = nw;                                                // left of edge 
      if (hf > 0)
      {      
        ne = s[ref + 101];
        if (ne == 0xFFFF)                                      // quad partially invalid
          continue;
        if (abs(ne - nw) < vjump)
          top = ((nw << 8) + hf * (ne - nw)) >> 8;             // full mix 
        else if (hf >= 128)
          top = ne;                                            // right of edge
      }
      
      // combine interpolated bottom with interpolated top
      pel = bot;                                               // lower half
      if (abs(top - bot) < hjump)
        pel = ((bot << 8) + vf * (top - bot)) >> 8;            // four way blend
      else if (vf >= 128)
        pel = top;                                             // upper half
      *d = (unsigned short) pel;
    }
  } 
}


// ----------------------------- GRAYSCALE ------------------------------- 

//= Convert full 4xmm depth image into monochrome version (dark = far).
// linearly maps value so lo16 goes to hi8, hi16 goes to lo8
// 0 = unknown, else 1-255 = grayscale depth in range (saturates)
// derived from jhcLUT::Remap16()

void remap_16 (unsigned char *d8, const unsigned short *d16, int lo16, int hi16, int lo8, int hi8) 
{
  int bot, top, x, y, f, vsc;
  const unsigned short *s = d16;
  unsigned char *d = d8;

  // get integer conversion coefficient
  bot = __max(0, __min(lo16, 65535));
  top = __max(0, __min(hi16, 65535));
  if (top == bot)
  {
    memset(d8, 0, 640 * 480);
    return;
  }
  f = ((hi8 - lo8) << 16) / (top - bot);

  // convert pixels
  for (y = 480; y > 0; y--)
    for (x = 640; x > 0; x--, d++, s++)
      if (*s > 40000)
        *d = 0;
      else
      {
        vsc = (f * (*s - bot) + 32768) >> 16;
        vsc = __max(0, __min(vsc, 254));
        *d = (unsigned char)(255 - vsc);
      }
}


//= Convert 16 bit pixels in range avg +/- k * sdev to 8 bits (hi vals -> dark).
// derived from jhcLUT::NightSD()

void night_sd (unsigned char *d8, const unsigned short *src, double sdf) 
{
  uint64_t sum = 0, ssq = 0;
  double avg, sdev;
  int x, y, lo16, hi16, bot = 65535, top = 0, n = 0;
  const unsigned short *v = src;

  // gather statistical data (ignores over 40000)
  for (y = 480; y > 0; y--)
    for (x = 640; x > 0; x--, v++)
      if (*v <= 40000)
      {
        top = __max(top, *v);
        bot = __min(bot, *v);
        sum += *v;
        ssq += (*v) * (*v);
        n++;
      }

  // special case with no valid depths
  if (n <= 0)
  {
    memset(d8, 0, 640 * 480);
    return;
  }

  // crunch into average and standard deviation 
  avg = (double) sum;
  avg /= (double) n;
  sdev = (double) ssq;
  sdev /= (double) n;
  sdev = sqrt(sdev - avg * avg);

  // linearly scale pixels (ignore very high sdf values)
  lo16 = (int)(avg - sdf * sdev + 0.5);
  hi16 = (int)(avg + sdf * sdev + 0.5);
  remap_16(d8, src, __max(bot, lo16), __min(hi16, top), 1, 255);
}


// ------------------------------- SAVING -------------------------------- 

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


//= Save a 640x480 8 bit BMP (DIB) format image.
// derived from jhcImgIO::SaveBmp()

void save_gray (const char *fname, const unsigned char *buf)
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
  write32(out, 308278);              // whole file size (14 + 40 + 1024 + 640 * 480)
  write32(out, 0);                   // Reserved 1 + 2
  write32(out, 1078);                // combined size of both headers

  // write the bitmapinfo header (40 bytes)
  write32(out, 40);                  // size of this header
  write32(out, 640);                 // image width
  write32(out, 480);                 // image height (bottom up)
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
  fwrite(buf, 1, 640 * 480, out);
  fclose(out);
}


//= Writes 16 bit value, takes care of big-endian/little-endian problems.
// this version assumes stores values in big-endian fashion

void write16b (FILE *out, unsigned short val)
{
  putc((val >> 8) & 0xFF, out);
  putc(val & 0xFF, out);
}


//= Writes 32 bit value, takes care of big-endian/little-endian problems.
// this version assumes stores values in big-endian fashion

void write32b (FILE *out, unsigned long val)
{
  write16b(out, (unsigned short)((val >> 16) & 0xFFFF));
  write16b(out, (unsigned short)(val & 0xFFFF));
}


//= Save a 640x480 16 bit Sun Raster format image.
// derived from jhcImgIO::SaveRas()

void save_d16 (const char *fname, const unsigned short *buf)
{
  FILE *out;
  const unsigned short *s, *s0 = buf + 479 * 640; 
  int x, y;

  // sanity check then try opening file
  if ((buf == NULL) || (fname == NULL) || (*fname == '\0'))
    return;
  if ((out = fopen(fname, "wb")) == NULL)
    return;

  // generate proper header
  write32b(out, 0x59A66A95);
  write32b(out, 640);
  write32b(out, 480);
  write32b(out, 16);
  write32b(out, 640 * 480 * 2);        
  write32b(out, 1);
  write32b(out, 0);
  write32b(out, 0);

  // write out pixels changing bottom up to top down scan
  for (y = 480; y > 0; y--, s0 -= 640)
  {
    s = s0;
    for (x = 640; x > 0; x--, s++)
    {
      putc(*s & 0xFF, out);
      putc((*s >> 8) & 0xFF, out);
    }                
  }
  fclose(out);
}


// -------------------------------- MAIN --------------------------------- 

//= Continuously show sensor and processed images until Ctrl-C.
// saves most recent 16 bit depth image and grayscale version at end

int main (int argc, char *argv[])
{
  jhcTofCam tof;
  cv::Mat flip;
  const unsigned char *buf;

  // build sampling arrays and try to start up sensor
  tof_sampling();
  if (tof.Start() <= 0)
  {
    printf("Could not connect to TOF sensor!\n");
    return -1;
  }

  // make display window
  cv::namedWindow("Expanded");
  cv::moveWindow("Expanded", 10, 10);

  // keep grabbing frames from sensor
  printf("Hit any key (with display window selected) to quit ...\n");
  while ((buf = tof.Range(1)) != NULL)
  {
    // produce larger sized images
    set_z16_tof(vga, buf);
    night_sd(gray.data, vga, 2.0);

    // display new frame and pump update message
    cv::flip(gray, flip, 0);
    cv::imshow("Expanded", flip);         
    if ((int) cv::waitKey(1) > 0)
      break;                    
  }

  // save most recent images on exit
  tof.Done();
  cv::destroyWindow("Expanded");
  save_d16("vga_z.ras", vga);
  save_gray("vga.bmp", gray.data);
  printf("Saved images vga_z.ras and vga.bmp\n");
  return 0;
}

