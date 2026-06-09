// jhc_pthread.h - portable version of threads, mutexes, and semaphores
// 
// Written by Jonathan H. Connell, jconnell@alum.mit.edu
// 
///////////////////////////////////////////////////////////////////////////
//
// Copyright 2023-2024 Etaoin Systems
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

#ifndef __linux__

#include <windows.h>                   // for HANDLE
#include <process.h>
#include <synchapi.h>

// --------------------------------- thread -----------------------------------

// type of a thread identifier
#define pthread_t HANDLE

// start a new thread running tfcn(arg)
#define pthread_create(p_th, attr, tfcn, arg)  \
  *p_th = (HANDLE) _beginthreadex(NULL, 0, tfcn, arg, 0, NULL)

// return value for a thread function (non-standard)
#define pthread_ret unsigned int __stdcall

// check if thread has terminated (non-standard)
#define pthread_busy(th) (WaitForSingleObject(th, 0) != WAIT_OBJECT_0)

// wait for thread to terminate
#define pthread_join(th, ret) WaitForSingleObject(th, INFINITE)

// wait only a certain amount of time for termination
#define pthread_timedjoin_np(th, ret, p_wait) (WaitForSingleObject(th, *p_wait) != WAIT_OBJECT_0)

// deallocate a thread
#define pthread_detach(th) th = NULL

// --------------------------------- timeout ----------------------------------

// type of a wait time specification (non-standard)
#define abstime_t int

// setup to wait some milliseconds from now (non-standard)
inline abstime_t *abstime_wait (abstime_t *p_ts, int ms) 
  {*p_ts = ms; return p_ts;}

// --------------------------------- mutex ------------------------------------

// type of a mutex identifier 
#define pthread_mutex_t HANDLE

// initialize a mutex (ignores attributes)
#define pthread_mutex_init(p_mut, attr) *p_mut = CreateMutex(NULL, FALSE, NULL)

// grab control of a mutex
#define pthread_mutex_lock(p_mut) WaitForSingleObject(*p_mut, INFINITE)

// release control of a mutex
#define pthread_mutex_unlock(p_mut) ReleaseMutex(*p_mut)

// grab control of a mutex or fail immediately
#define pthread_mutex_trylock(p_mut) \
  ((WaitForSingleObject(*p_mut, 0) == WAIT_OBJECT_0) ? 0 : -1)

// grab control of a mutex or fail on timeout
#define pthread_mutex_timedlock(p_mut, p_wait)  \
  ((WaitForSingleObject(*p_mut, *p_wait) == WAIT_OBJECT_0) ? 0 : -1)

// deallocate a mutex
#define pthread_mutex_destroy(p_mut)  \
  {if (*p_mut != INVALID_HANDLE_VALUE) CloseHandle(*p_mut);}

// ------------------------------- semaphores ---------------------------------

// type of a semaphore identifier 
#define sem_t HANDLE

// initialize a semaphore (ignores sharing attribute)
#define sem_init(p_sem, sh, val)  \
  *p_sem = CreateSemaphore(NULL, val, __max(1, val), NULL)

// set (increment) a semaphore
#define sem_post(p_sem) ReleaseSemaphore(*p_sem, 1, NULL)

// wait until set then clear (decrement) a semaphore
#define sem_wait(p_sem) WaitForSingleObject(*p_sem, INFINITE)

// wait until set then clear (decrement) a semaphore or fail on timeout
#define sem_timedwait(p_sem, p_ts) WaitForSingleObject(*p_sem, *p_ts)

// deallocate a semaphore
#define sem_destroy(p_sem)  \
  {if (*p_sem != INVALID_HANDLE_VALUE) CloseHandle(*p_sem);}

#else

#include <pthread.h>
#include <semaphore.h>
#include <errno.h>
#include <time.h>

// return value for a thread function (non-standard)
#define pthread_ret void *

// check if thread has terminated (non-standard)
#define pthread_busy(th) (pthread_tryjoin_np(th, NULL) == EBUSY)

// type of a wait time specification (non-standard)
#define abstime_t timespec

// setup to wait some milliseconds from now (non-standard)
inline abstime_t *abstime_wait (abstime_t *p_ts, int ms)
{  
  clock_gettime(CLOCK_REALTIME, p_ts); 
  p_ts->tv_sec += ms / 1000; 
  p_ts->tv_nsec += 1000000 * (ms % 1000);
  if (p_ts->tv_nsec >= 1000000000)
  {
    p_ts->tv_nsec -= 1000000000;
    p_ts->tv_sec++;
  }
  return p_ts;
}

#endif