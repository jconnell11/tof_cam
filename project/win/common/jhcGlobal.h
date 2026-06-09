// jhcGlobal.h : placeholder for any global includes required
//
// Written by Jonathan H. Connell, jconnell@alum.mit.edu
//
///////////////////////////////////////////////////////////////////////////
//
// Copyright 2006-2012 IBM Corporation
// Copyright 2022-2024 Etaoin Systems
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

// Note that this file can be copied and customized for each project.
// all JHC class header files should include this file as the first thing

#pragma once


///////////////////////////////////////////////////////////////////////////

// operating system & compiler changes

#ifndef __linux__

  #include <stdlib.h>                  // for __min and NULL

//  #define JHC_MMX                      // intrinsics are Windows style

#else  // **** FOR PORTING (Linux mostly) ****

  #include <stdlib.h>                  // for NULL
  #include <unistd.h>
  #include <sys/stat.h>
  #include <sys/types.h>

  // minimum and maximum
  #define __min(a, b) (((a) < (b)) ? (a) : (b))
  #define __max(a, b) (((a) > (b)) ? (a) : (b))

  // string comparison
  #define _stricmp(a, b)     strcasecmp(a, b)
  #define _strnicmp(a, b, n) strncasecmp(a, b, n)

  // file system functions
  #define _chdir(path) chdir(path)
  #define _mkdir(path) mkdir(path, S_IRWXU | S_IRWXG | S_IRWXO)
  #define _fsopen(name, mode, share) fopen(name, mode)           

  // safe string functions
  #include "jhc_str_s.h"

#endif


///////////////////////////////////////////////////////////////////////////

#include <stdint.h>

// these datatypes are for pixels and packed pixels (bytes)
// some DSPs define long as 40 bits rather than 32 bits

typedef uint8_t  UC8;
typedef uint16_t US16;
typedef uint32_t UL32;
typedef uint64_t UL64; 


//= The standard value for pi.
#ifndef PI
  #define PI  3.14159265358979323846
#endif


//= Conversion factor for radians to degrees. 
#define R2D  (180.0 / PI)  


//= Conversion factor for degrees to radians. 
#define D2R  (PI / 180.0)


//= Limit some value to range of 0 to 255.
#define BOUND(v)  (UC8)(__min(255, __max(0, (v))))


//= Very common macro for rounding.
#define ROUND(v)  (int)((v) + 0.5)


///////////////////////////////////////////////////////////////////////////

// useful debugging routines in common video directory (pre-2024 code)

//#include "Interface/jprintf.h"
//#include "Interface/jtimer.h"


// special directives for use with JHC libraries
// these can also be defined as command line definitions 

//#define JHC_GVID
//#define JHC_JPEG
//#define JHC_TIFF

