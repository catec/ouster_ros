/*!
 *      @file  packets_queue.h
 *    @author  Rafael Caballero González (RCG), rcaballero@catec.aero
 *
 *  @internal
 *    Created  12/11/2020
 *   Compiler  gcc/g++
 *    Company  FADA-CATEC
 *  Copyright (c) 2020, FADA-CATEC
 */

#pragma once

#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>

#include "ouster_ros/PacketMsg.h"

namespace ouster_ros
{
template <class T>
class PacketsQueue
{
  public:
   PacketsQueue(std::string queue_name, int max_size = 2048);
   ~PacketsQueue();

   void add(T& packet);

   T get();

   int size();

  private:
   std::condition_variable _wait_cond;
   std::mutex _mutex;
   std::deque<T> _queue;

   std::string _name;
   size_t _max_size;
};

template class PacketsQueue<PacketMsg>;
}  // namespace ouster_ros