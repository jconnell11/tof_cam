// jhcSerial.h : serial port interface
//
// Written by Jonathan H. Connell, jconnell@alum.mit.edu
//
///////////////////////////////////////////////////////////////////////////
//
// Copyright 2002-2019 IBM Corporation
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

#pragma once

#include <windows.h>

#include "jhcGlobal.h"


//= Serial port interface.

class jhcSerial
{
// PRIVATE MEMBER VARIABLES
private:
  DCB dcb;
  HANDLE sport;
  int snum, last;


// PUBLIC MEMBER VARIABLES
public:
  double wtime, btime;


// PUBLIC MEMBER FUNCTIONS
public:
  // creation and destruction
  ~jhcSerial ();
  jhcSerial ();
  jhcSerial (int port, int baud =9600, int bits =8, int stop =1, int parity =0);

  // configuration
  int SetSource (int port, int baud =9600, int bits =8, int stop =1, int parity =0);
  void SetTimeouts (double wait =0.1, double barf =0.2);
  int Close ();

  // report properties
  int Valid ();
  int PortNum ();
  int Baud ();
  int DataBits ();
  int StopBits ();
  int Parity ();

  // basic operations
  int Rcv ();
  int Xmit (int val);
  int RxLine (char *dest, int len =40, char end ='\0', int mask =0xFF); 
  int TxLine (const char *line);
  int RxArray (UC8 *dest, int n);
  int TxArray (UC8 *src, int n);
  int Check ();
  int Flush (int pause =1);
  void SetDTR (int val =1);
  void SetRTS (int val =1);
  int Handshake ();


// PRIVATE MEMBER FUNCTIONS
private:
  void init_vals ();

};

