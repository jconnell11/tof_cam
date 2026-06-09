// jhcSerial.cpp : serial port (RS-232) communication routines
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

#include <stdio.h>
#include <tchar.h>

#include "Interface/jms_x.h"

#include "Interface/jhcSerial.h"


///////////////////////////////////////////////////////////////////////////
//                         Creation and Destruction                      //
///////////////////////////////////////////////////////////////////////////

//= Default destructor makes sure port is closed.

jhcSerial::~jhcSerial ()
{
  Close();
}


//= Default constructor does nothing really.

jhcSerial::jhcSerial ()
{
  dcb.DCBlength = sizeof(DCB);
  init_vals();
}


//= More specific constructor tries to establish a connection.
// parity: 0 = none, 1 = odd, 2 = even

jhcSerial::jhcSerial (int port, int baud, int bits, int stop, int parity)
{
  init_vals ();
  SetSource(port, baud, bits, stop, parity);
}


//= Setup default member variables.

void jhcSerial::init_vals ()
{
  snum = 0;
  last = -1;
  sport = INVALID_HANDLE_VALUE;
  SetTimeouts();
}


///////////////////////////////////////////////////////////////////////////
//                              Configuration                            //
///////////////////////////////////////////////////////////////////////////

//= Configure port as requested.
// parity: 0 = none, 1 = odd, 2 = even
// return 1 if okay, 0 if bad request, -1 if some error

int jhcSerial::SetSource (int port, int baud, int bits, int stop, int parity)
{
	COMMTIMEOUTS timeouts;
  int rate[15] = { 256000, 230400, 128000,        // Windows limited to 256K baud
                   115200,  57600,  38400,  19200,  14400,  9600, 
                     4800,   2400,   1200,    600,    300,   110};
  TCHAR name[20];
  int i;

  // check for valid arguments
  if ((port <= 0) || (port > 20))
    return 0;
  if ((baud < 100) || (baud > 1000000))
    return 0;

  // try opening requested port (if not already bound) and get params
  if (!Valid() || (snum != port))
    Close();
  if (sport == INVALID_HANDLE_VALUE)
  {
    snum = port;
    _stprintf_s(name, _T("\\\\.\\COM%d"), snum);  // needed for snum > 9
    sport = CreateFile((LPCTSTR) name, 
                       GENERIC_READ | GENERIC_WRITE,  // All rights
                       0,                 // Exclusive access; no sharing
                       NULL,              // Default security attributes
                       OPEN_EXISTING,     // Port must exist, of course
                       0,                 // Use synchronous access
                       NULL);             // No template file
    last = -1;
  }
  if (sport == INVALID_HANDLE_VALUE)
    return Close();
	if (!GetCommState(sport, &dcb))
	  return Close();
  
  // set new canonical baud rate 
  dcb.BaudRate = 9600;
  for (i = 0; i < 15; i++)                // Windows limited to 256K baud
    if (baud >= rate[i])
    {
      dcb.BaudRate = rate[i];
      break;
    }

  // set number of data bits and stop bits
  if (bits <= 7)
  	dcb.ByteSize	= 7;
  else
  	dcb.ByteSize	= 8;
  if (stop <= 1)
    dcb.StopBits = ONESTOPBIT;
  else 
    dcb.StopBits = TWOSTOPBITS;

  // set parity
	dcb.fParity	= TRUE;
  if (parity <= 0)
  	dcb.fParity	= FALSE;
  else if (parity == 1)
    dcb.Parity = ODDPARITY;
  else 
    dcb.Parity = EVENPARITY;

  // disable XON/XOFF and RTS/CTS 
	dcb.fOutX	 	        = FALSE;
	dcb.fInX		        = FALSE;
  dcb.fRtsControl     = RTS_CONTROL_ENABLE;  
  dcb.fOutxCtsFlow    = FALSE;
  dcb.fDtrControl     = DTR_CONTROL_ENABLE;
  dcb.fOutxDsrFlow    = FALSE; 
  dcb.fDsrSensitivity = FALSE; 

  // upload communication parameters
	if(!SetCommState(sport, &dcb))
		return Close();

	// setup ReadFile calls to return immediately with whatever 
	// data is available, even if no data is available at all
	timeouts.ReadIntervalTimeout         = MAXDWORD;
	timeouts.ReadTotalTimeoutMultiplier	 = 0;
	timeouts.ReadTotalTimeoutConstant    = 0;
	timeouts.WriteTotalTimeoutMultiplier = 0;
	timeouts.WriteTotalTimeoutConstant	 = 0;

  // upload timing parameters
	if(!SetCommTimeouts(sport, &timeouts))
    return Close();
  return 1;	
}


//= Set how many seconds to wait for various transactions.
// wait = how long to wait before giving up on receive (e.g. 0.1 seconds) 
// barf = how long to wait before purging receive buffer (e.g. 0.2 seconds) 

void jhcSerial::SetTimeouts (double wait, double barf)
{
  wtime = __max(0.0, wait);
  btime = __max(0.0, barf);
}


//= Release port source immediately.

int jhcSerial::Close ()
{
  if (sport != INVALID_HANDLE_VALUE)
    CloseHandle(sport);
  init_vals();
  return -1;
}


///////////////////////////////////////////////////////////////////////////
//                           Report Properties                           //
///////////////////////////////////////////////////////////////////////////

//= Whether anything is connected.

int jhcSerial::Valid ()
{
  if (sport == INVALID_HANDLE_VALUE)
    return 0;
  return 1;
}


//= What port is being used.

int jhcSerial::PortNum ()
{
  if (!Valid())
    return -1;
  return snum;
}


//= Communications speed.

int jhcSerial::Baud ()
{
  if (!Valid())
    return -1;
	if (!GetCommState(sport, &dcb))
	  return Close();
  return dcb.BaudRate;
}


//= Number of bits per byte.

int jhcSerial::DataBits ()
{
  if (!Valid())
    return -1;
	if (!GetCommState(sport, &dcb))
	  return Close();
  return dcb.ByteSize;
}


//= Stop bits sent and expected.

int jhcSerial::StopBits ()
{
  if (!Valid())
    return -1;
	if (!GetCommState(sport, &dcb))
	  return Close();
  if (dcb.StopBits == TWOSTOPBITS)
    return 2;
  return 1;
}


//= What parity scheme is used (0 = none, 1 = odd, 2 = even).

int jhcSerial::Parity ()
{
  if (!Valid())
    return -1;
	if (!GetCommState(sport, &dcb))
	  return Close();
  if (dcb.fParity == 0)
    return 0;
  if (dcb.Parity == ODDPARITY)
    return 1;
  return 2;
}


///////////////////////////////////////////////////////////////////////////
//                           Basic Operations                            //
///////////////////////////////////////////////////////////////////////////

//= Get serial character, wait if none received yet.
// -1 is stream error, -2 is timeout, else byte    

int jhcSerial::Rcv ()
{
  int tmp;
  DWORD len, end;
  UC8 val;

  // make sure port is open and check for any saved data
  if (!Valid())
    return -1;
  if (last >= 0)
  {
    tmp = last;
    last = -1;
    return tmp;
  }

  // keep trying to read the port for a while
  end = jms_now() + (DWORD)(wtime * 1000.0 + 0.5);
  while (1)
  {
    tmp = ReadFile(sport, &val, 1, &len, NULL);
    if ((tmp != 0) && (len > 0))
      break;
    if (jms_now() >= end)
      return -2;
    jms_sleep(1);
  }
  return((int) val);
}


//= Send out a serial character and wait for completion.

int jhcSerial::Xmit (int val)
{
  DWORD len;
  UC8 b;

  if (!Valid())
    return 0;
  b = (UC8)(val & 0xFF);
  if (!WriteFile(sport, &b, 1, &len, NULL))
    return -1;
  return 1;
}


//= Received up to len characters into provided dest string.
// stops when at end of string or receives the end character
// always terminates string with null character
// can optionally reduce number of bits in characters
// returns number of characters in string that were filled
// Note: call blocks until end character is received

int jhcSerial::RxLine (char *dest, int len, char end, int mask)
{
  int ch, i = 0, lim = len - 1;

  for (i = 0; i < lim; i++)
  {
    while (Check() <= 0)
      jms_sleep(1);
    ch = Rcv() & mask;
    if (ch == end)
      break;
    dest[i] = (char) ch;
  }
  dest[i] = '\0';
  return i;
}   


//= Send a string of characters to device.
// returns number of characters sent (may timeout)

int jhcSerial::TxLine (const char *line)
{
  DWORD len;

  if (!Valid())
    return 0;
  if (!WriteFile(sport, (UC8 *) line, (DWORD) strlen(line), &len, NULL))
    return -1;
  return len;
}


//= Receive a fixed number of bytes from the port.
// returns number of bytes actually received (may timeout = -2)
// Note: call blocks until correct number received or timeout

int jhcSerial::RxArray (UC8 *dest, int n)
{
  DWORD start, len;
  int wait = ROUND(1000.0 * wtime), got = 0;

  if (!Valid())
    return 0;

  // keep trying to read the port for a while
  start = jms_now();
  while (1)
  {
    // read some (possibly not all) bytes from port
    if (!ReadFile(sport, dest + got, n - got, &len, NULL))
      return -1;
    got += len;
    if (got >= n)
      break;

    // see if timeout expired else try again in a while
    if (jms_diff(jms_now(), start) > wait)
      return -2;
    jms_sleep(1);
  }
  return got;
}


//= Send a fixed number of bytes to the port.
// returns number of bytes sent (may timeout)

int jhcSerial::TxArray (UC8 *src, int n)
{
  DWORD len;

  if (!Valid())
    return 0;
  if (!WriteFile(sport, src, n, &len, NULL))
    return -1;
  FlushFileBuffers(sport);
  return len;
}


//= See if any data has been received (1 = yes, 0 = no).
// does an actual read and stores received byte (if any)

int jhcSerial::Check ()
{
  int tmp;
  DWORD len;
  UC8 val;

  if (!Valid())
    return 0;
  if (last >= 0)
    return 1;
  tmp = ReadFile(sport, &val, 1, &len, NULL);
  if ((tmp == 0) || (len == 0))
    return 0;
  last = (int) val;
  return 1;
}


//= Pause a while then discard any received characters.

int jhcSerial::Flush (int pause)
{
  if (!Valid())
    return 0;
  if (pause > 0)
    jms_sleep((int)(btime * 1000.0 + 0.5));
  PurgeComm(sport, PURGE_RXCLEAR);
  last = -1;
  return 1;
}


//= Set the Data Terminal Ready handshake signal (pin 4 on DB-9).

void jhcSerial::SetDTR (int val)
{
  EscapeCommFunction(sport, ((val > 0) ? SETDTR : CLRDTR));
}


//= Set the Ready To Send handshake signal (pin 7 on DB-9).

void jhcSerial::SetRTS (int val)
{
  EscapeCommFunction(sport, ((val > 0) ? SETRTS : CLRRTS));
}


//= Get input line from port as a bit string (DCD : RING : DSR : CTS).
// on DB-9 = pin 1 : pin 9 : pin 6 : pin 8

int jhcSerial::Handshake ()
{
  DWORD state = 0;

  GetCommModemStatus(sport, &state);
  return((state >> 4) & 0xFF);
}

