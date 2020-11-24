/*!
 *      @file  packets_queue.cpp
 *    @author  Rafael Caballero González (RCG), rcaballero@catec.aero
 *
 *  @internal
 *    Created  12/11/2020
 *   Compiler  gcc/g++
 *    Company  FADA-CATEC
 *  Copyright (c) 2020, FADA-CATEC
 */

#include <ros/ros.h>

#include "ouster_ros/packets_queue.h"

namespace ouster_ros
{
template <class T>
PacketsQueue<T>::PacketsQueue(std::string name, int max_size)
{
   _name     = name;
   _max_size = max_size;
}

template <class T>
PacketsQueue<T>::~PacketsQueue()
{
}

template <class T>
void PacketsQueue<T>::add(T& data)
{
   {
      std::unique_lock<std::mutex> mutex_guard(_mutex);

      if (_queue.size() < _max_size)
         _queue.push_back(std::move(data));
      else
         std::cout << "[" << _name << "]"
                   << "Cannot add packet. Queue is full !" << std::endl;
   }

   _wait_cond.notify_all();
}

template <class T>
T PacketsQueue<T>::get()
{
   std::unique_lock<std::mutex> mutex_guard(_mutex);

   while (_queue.empty())
   {
      _wait_cond.wait(mutex_guard);
   }

   T data = std::move(_queue.front());
   _queue.pop_front();

   return data;
}

template <class T>
int PacketsQueue<T>::size()
{
   std::unique_lock<std::mutex> mutex_guard(_mutex);

   return _queue.size();
}

}  // namespace ouster_ros