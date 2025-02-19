/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2020 TELEMATICS LAB, Politecnico di Bari
 *
 * This file is part of 5G-air-simulator
 *
 * 5G-air-simulator is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation;
 *
 * 5G-air-simulator is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with 5G-air-simulator; if not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Telematics Lab <telematics-dev@poliba.it>
 */




#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "event.h"
#include "calendar.h"
#include "make-event.h"

#include <stdint.h>
#include <string>
#include <iostream>

#include "../../load-parameters.h"

/*
 * Simulator
 * Control the scheduling of simulation events.
 */

class Simulator
{
private:
  Simulator ();
  static Simulator *ptr;
  Calendar *m_calendar;
  bool m_stop;
  unsigned long long int m_currentUid;
  unsigned long long int m_lastAssignedUid;
  double m_currentTs;
  int m_unscheduledEvents;

  int m_uid;

  void ProcessOneEvent (void);

public:
  virtual ~Simulator ();

  static Simulator*
  Init (void)
  {
    if (ptr==nullptr)
      {
        ptr = new Simulator;
      }
    return ptr;
  }

  double Now (void);
  void Reset(void);
  void Run (void);
  void Stop (void);
  void SetStop (double time);

  int GetUID (void);

  shared_ptr<Event>
  DoSchedule (double time, shared_ptr<Event> event);


  /*
   * Schedule methods are called to insert a new method
   * into the calendar scheduler
   */
  template <typename MEM, typename OBJ>
  shared_ptr<Event>
  Schedule (double time, MEM mem_ptr, OBJ obj);

  template <typename MEM, typename OBJ, typename T1>
  shared_ptr<Event>
  Schedule (double time, MEM mem_ptr, OBJ obj, T1 a1);

  template <typename MEM, typename OBJ, typename T1, typename T2>
  shared_ptr<Event>
  Schedule (double time, MEM mem_ptr, OBJ obj, T1 a1, T2 a2);

  template <typename MEM, typename OBJ, typename T1, typename T2, typename T3>
  shared_ptr<Event>
  Schedule (double time, MEM mem_ptr, OBJ obj, T1 a1, T2 a2, T3 a3);

  shared_ptr<Event>
  Schedule (double time, void (*f) (void));

  template <typename U1>
  shared_ptr<Event>
  Schedule (double time, void (*f) (U1));

  template <typename U1, typename T1>
  shared_ptr<Event>
  Schedule (double time, void (*f) (U1), T1 a1);

  void
  PrintMemoryUsage (void);
};


template <typename MEM, typename OBJ>
shared_ptr<Event>
Simulator::Schedule (double time, MEM mem_ptr, OBJ obj)
{
  return DoSchedule (time, MakeEvent (mem_ptr, obj));
}

template <typename MEM, typename OBJ, typename T1>
shared_ptr<Event>
Simulator::Schedule (double time, MEM mem_ptr, OBJ obj, T1 a1)
{
  return DoSchedule (time, MakeEvent(mem_ptr, obj, a1));
}

template <typename MEM, typename OBJ, typename T1, typename T2>
shared_ptr<Event>
Simulator::Schedule (double time, MEM mem_ptr, OBJ obj, T1 a1, T2 a2)
{
  return DoSchedule (time, MakeEvent(mem_ptr, obj, a1, a2));
}

template <typename MEM, typename OBJ, typename T1, typename T2, typename T3>
shared_ptr<Event>
Simulator::Schedule (double time, MEM mem_ptr, OBJ obj, T1 a1, T2 a2, T3 a3)
{
  return DoSchedule (time, MakeEvent(mem_ptr, obj, a1, a2, a3));
}

/*
shared_ptr<Event>
Simulator::Schedule (double time, void (*f) (void))
{
  //NON FUNZIONA ANCORA !?!?!?!
  //return DoSchedule (time, MakeEvent (f));
}
*/

template <typename U1, typename T1>
shared_ptr<Event>
Simulator::Schedule (double time, void (*f) (U1), T1 a1)
{
  return DoSchedule (time, MakeEvent(f, a1));
}




#endif /* SIMULATOR_H */


