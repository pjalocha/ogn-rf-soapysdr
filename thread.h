/*
    OGN - Open Glider Network - http://glidernet.org/
    Copyright (c) 2015 The OGN Project

    A detailed list of copyright holders can be found in the file "AUTHORS".

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this software.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __THREAD_H__
#define __THREAD_H__

#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>

#include <pthread.h>
#include <sys/types.h>
#include <sys/syscall.h>
#include <errno.h>

#include <queue>
#include <algorithm>

// ======================================================================================

class MutEx   // for Mutual Exclusive access to a resource
{ private:
   pthread_mutex_t muLock;

  public:
   MutEx() { Init(); }
  ~MutEx() { Destroy(); }

  private:
   int Init(void)     { return pthread_mutex_init(&muLock, 0); }
   int Destroy(void)  { return pthread_mutex_destroy(&muLock); }

  public:
   int Lock(void)     { return pthread_mutex_lock(&muLock); }        // gain access to the object (can block)
   int Unlock(void)   { return pthread_mutex_unlock(&muLock); }      // release the access to the object
   int TryLock(void)  { int Err=pthread_mutex_trylock(&muLock);      // non-blocking attempt to lock
                        return Err==EBUSY ? 1:Err; }                 // 0 => lock succesfull, 1 => already locked by someone
} ;

// ======================================================================================

class Condition
{ private:
   pthread_cond_t      sigCond;
   pthread_condattr_t  Attr;
   pthread_mutex_t     muLock;

 public:
  Condition() { Init(); }
 ~Condition() { Destroy(); }

  private:
   int Init(void)     { pthread_condattr_init(&Attr);
#ifndef __MACH__ // _POSIX_TIMERS
                        pthread_condattr_setclock( &Attr, CLOCK_MONOTONIC); // use monotonic clock for timed wait
#endif
                        if(pthread_cond_init(&sigCond, &Attr)!=0) return -1;
                        return pthread_mutex_init(&muLock, 0); }
   int Destroy(void)  { int Err=pthread_mutex_destroy(&muLock); if(Err!=0) return Err;
                            Err=pthread_cond_destroy(&sigCond); if(Err!=0) return Err;
                        pthread_condattr_destroy(&Attr); return 0; }

  public:
   int Lock(void)      { return pthread_mutex_lock(&muLock); }           // gain access to the object (can block)
   int Unlock(void)    { return pthread_mutex_unlock(&muLock); }         // release the access to the object
   int TryLock(void)   { int Err=pthread_mutex_trylock(&muLock);         // non-blocking attempt to lock
                         return Err==EBUSY ? 1:Err; }                    // 0 => lock succesfull, 1 => already locked by someone

   int Signal(void)    { return pthread_cond_signal(&sigCond); }         // signal to at least one thread, that something has changed
   int Broadcast(void) { return pthread_cond_broadcast(&sigCond); }      // signal to all threads, that something has changed
   int Wait(void)      { return pthread_cond_wait(&sigCond, &muLock); }  // wait for a signal (can block)
                                                                         // mutex must be locked while waiting
#ifndef __MACH__ // _POSIX_TIMERS       // OSX has no clock_gettime() thus we need a different code here
   int TimedWait(int usec)                                              // wait, but no longer than [usec]
   { timespec Now; clock_gettime(CLOCK_MONOTONIC, &Now);
                               // printf("clock_gettime(%ld.%06ld)\n", (long)(Now.tv_sec), (long)(Now.tv_nsec) );
     timespec StopTime;
     int sec=usec/1000000; usec-=sec*1000000; int nsec=usec*1000;
                               // printf("%ld.%06ld = %ld.%09ld\n", sec, usec, sec, nsec);
     StopTime.tv_sec  = Now.tv_sec  + sec;
     StopTime.tv_nsec = Now.tv_nsec + nsec;
     if(StopTime.tv_nsec>1000000000) { StopTime.tv_nsec-=1000000000; StopTime.tv_sec+=1; }
                               // printf("StopTime = %ld.%09ld\n", (long)(StopTime.tv_sec), (long)(StopTime.tv_nsec));
     return TimedWait(StopTime); }
   int TimedWait(struct timespec &StopTime)                              // wait, but wakeup no later than [StopTime]
   { return pthread_cond_timedwait(&sigCond, &muLock, &StopTime); }
#endif

} ;

// ======================================================================================

template <class Type>
 class MessageQueue
{ public:
   std::queue<Type> Queue;
   Condition        Cond;

  public:

   int Push(Type Msg)
   { Cond.Lock();
     Queue.push(Msg); int Size=Queue.size();
     Cond.Unlock();
     Cond.Signal(); return Size; }

   int Pop(Type &Msg)
   { Cond.Lock();
     while(Queue.empty()) Cond.Wait();
     Msg = Queue.front();
     Queue.pop(); int Size=Queue.size();
     Cond.Unlock(); return Size; }

   Type Pop(void)
   { Cond.Lock();
     while(Queue.empty()) Cond.Wait();
     Type Msg = Queue.front();
     Queue.pop();
     Cond.Unlock(); return Msg; }

   int Size(void)
   { Cond.Lock();
     int size = Queue.size();
     Cond.Unlock();
     return size; }

} ;

template <class Type>
 class ObjectQueue                // this object queue holds objects
{ public:                         //
   std::queue<Type *> Queue;      // objects in the queue
   Condition          Cond;

  public:

   ObjectQueue()
   { }

  ~ObjectQueue()
   { while(!Queue.empty()) { delete Queue.front(); Queue.pop(); }
   }

   void Push(Type *Obj)                // add object to the queue
   { Cond.Lock();
     Queue.push(Obj);                  // push the object to the queue
     Cond.Unlock();
     Cond.Signal(); }

   Type *Pop(void)                     // take object from the queue (wait forever, if no object there)
   { Cond.Lock();
     while(Queue.empty()) Cond.Wait(); // wait for the queue not being empty
     Type *Obj = Queue.front(); Queue.pop();
     Cond.Unlock(); return Obj; }

   int Size(void)                      // how many objects waiting in the queue ?
   { Cond.Lock();
     int Size = Queue.size();
     Cond.Unlock();
     return Size; }

} ;

#define WITH_DEQUEUE

template <class Type>
 class ReuseObjectQueue           // this object queue holds objects
{ public:                         // that can be reused - thus don't need to be created and deleted all the time
#ifdef WITH_DEQUEUE
   std::deque<Type *> Queue;      // objects in the queue
#else
   std::queue<Type *> Queue;      // objects in the queue
#endif
   std::queue<Type *> Reuse;      // objects to be reused, these can be queued again
   Condition          Cond;
   int                Float;      // number of "floating" objects which are in neither queue

  public:

   ReuseObjectQueue()
   { Float=0; }

  ~ReuseObjectQueue()
   {
#ifdef WITH_DEQUEUE
     while(!Queue.empty()) { delete Queue.front(); Queue.pop_front(); }
#else
     while(!Queue.empty()) { delete Queue.front(); Queue.pop(); }
#endif
     while(!Reuse.empty()) { delete Reuse.front(); Reuse.pop(); }
   }

#ifdef WITH_DEQUEUE
   void Sort(bool (Less)(const Type *A, const Type *B))
   { Cond.Lock();
     std::sort(Queue.begin(), Queue.end(), Less);
     Cond.Unlock(); }
#endif

   Type *New(void)                     // create new (or re-use and old) object
   { Cond.Lock();
     Type *Obj;
     if(Reuse.empty())                 // if no objects which can be re-used
     { Obj = new Type; }               // create a new object and return it
     else                              // but if there are objects which can be reused
     { Obj = Reuse.front(); Reuse.pop(); } // then take them from the Reuse queue
     Float++;
     Cond.Unlock();
     return Obj; }                     // return pointer to the object which can be now used

   void Push(Type **Obj, int Num)       // add object(s) to the queue
   { Cond.Lock();
     for(int Idx=0; Idx<Num; Idx++)
     {
#ifdef WITH_DEQUEUE
       Queue.push_back(Obj[Idx]);      // push the object to the queue
#else
       Queue.push(Obj[Idx]);           // push the object to the queue
#endif
     }
     Float-=Num;
     Cond.Unlock();
     Cond.Signal(); }

   void Push(Type *Obj)                // add object to the queue
   { Cond.Lock();
#ifdef WITH_DEQUEUE
     Queue.push_back(Obj); Float--;         // push the object to the queue
#else
     Queue.push(Obj); Float--;         // push the object to the queue
#endif
     Cond.Unlock();
     Cond.Signal(); }

#ifdef WITH_DEQUEUE
   void RePush(Type *Obj)
   { Cond.Lock();
     Queue.push_front(Obj); Float--;
     Cond.Unlock(); }
#endif

   Type *Pop(void)                     // take object from the queue (wait forever, if no object there)
   { Cond.Lock();
     while(Queue.empty()) Cond.Wait(); // wait for the queue not being empty
#ifdef WITH_DEQUEUE
     Type *Obj = Queue.front(); Queue.pop_front(); Float++;
#else
     Type *Obj = Queue.front(); Queue.pop(); Float++;
#endif
     Cond.Unlock(); return Obj; }

   void Recycle(Type *Obj)                   // call when object no longer needed
   { Cond.Lock();
     // if( Reuse.size()<=(4+Queue.size()) ) // decide: reuse or delete
     if( Reuse.size()<(2*Queue.size()+4))    // decide: reuse or delete
     { Reuse.push(Obj); }                    //
     else
     { delete Obj; }
     Float--;
     Cond.Unlock();
     Cond.Signal(); }

   int Size(void)                      // how many objects waiting in the queue ?
   { Cond.Lock();
     int Size = Queue.size();
     Cond.Unlock();
     return Size; }

   int ReuseSize(void)               // how many objects waiting in the queue ?
   { Cond.Lock();
     int Size = Reuse.size();
     Cond.Unlock();
     return Size; }

   int FloatSize(void)
   { Cond.Lock();
     int Size = Float;
     Cond.Unlock();
     return Size; }

} ;

// ======================================================================================

class Lock    // for multiple read-access and exclusive write-access to a resource
{ private:
   pthread_rwlock_t rwLock;

  public:
   Lock() { Init(); }
  ~Lock() { Destroy(); }

   int Init(void)     { return pthread_rwlock_init(&rwLock, 0); }
   int Destroy(void)  { return pthread_rwlock_destroy(&rwLock); }

   int ReadLock(void)  { return pthread_rwlock_rdlock(&rwLock); } // lock for read-only
   int WriteLock(void) { return pthread_rwlock_wrlock(&rwLock); } // lock for read-write
   int Unlock(void)    { return pthread_rwlock_unlock(&rwLock); } // unlock

} ;

// ======================================================================================

inline int GetTID(void) { return syscall(SYS_gettid); }                       // return Thread ID (not Process ID) of the caller
inline int GetPID(void) { return getpid(); }                                  // return the process ID of the calling process
inline int GetParentPID(void) { return getppid(); }                           // return the process ID of the parent process

class Thread
{ private:
   pthread_t ID;
   void *(*Exec)(void *Context);

  public:
   Thread( void *(*Function)(void *) = 0) { ID=0; setExec(Function); }

   void setExec( void *(*Function)(void *)) { Exec=Function; } // set the function to run by this thread

   int Create(void *Context=0, const char *Name=0)             // create (start) this thread
   { if(Exec==0) return -1;                                    // Exec function not set
     if(ID) return -1;                                         // ID non-zero - a thread is running ?
     int Ret=pthread_create(&ID, 0, Exec, Context);
     if(!Ret && Name) pthread_setname_np(ID, Name);
     return Ret; }

   int Join(void)
   { void *ExitStatus;
     if(ID==0) return -1;                                      // ID zero - a thread is not running anymore ?
     int Ret=pthread_join(ID, &ExitStatus);
     ID=0; return Ret; }

   int Join(int Timeout)
   { struct timespec Time;
     Time.tv_sec=Timeout;
     Time.tv_nsec=0;
     void *ExitStatus;
     if(ID==0) return -1;                                      // ID zero - a thread is not running anymore ?
     int Ret=pthread_timedjoin_np(ID, &ExitStatus, &Time);
     if(Ret==0) ID=0;
     return Ret; }

   int Join(void *&ExitStatus)                                 // wait for the thread to terminate
   { if(ID==0) return -1;                                      // ID zero - a thread is NOT running anymore ?
     int Ret=pthread_join(ID, &ExitStatus);                    // give back its termination status
     ID=0; return Ret; }                                       // not clear how to behave if an error occures.

   int Cancel(void)                                            // request to cancel the thread at a calcallation point
   { if(ID==0) return -1;                                      // ID zero - a thread is NOT running anymore ?
     return pthread_cancel(ID); }

   int getMaxPriority(int Policy=SCHED_FIFO)
   { return sched_get_priority_max(Policy); }

   int setPriority(int Priority, int Policy=SCHED_FIFO)
   { struct sched_param params; params.sched_priority = Priority;
     return pthread_setschedparam(ID, Policy, &params); }

   int getPriority(int &Priority, int &Policy)
   { struct sched_param params;
     int Error=pthread_getschedparam(ID, &Policy, &params);
     if(Error>=0) Priority=params.sched_priority;
     return Error; }

// ------------------------------------------------------------------------------------
                                    // the following calls are by the running thread:
   static int CancelEnable(void)                               // I CAN be cancelled
   { int Old; return pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, &Old); }
   static int CancelDisable(void)                              // I CAN NOT be cancelled
   { int Old; return pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, &Old); }
   static void TestCancel(void)                                // test, if I was cancelled
   { pthread_testcancel(); }                                   // if I was: terminate

} ;

// ======================================================================================

#endif // of __THREAD_H__
