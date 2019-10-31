#ifndef __DATA_RECEIVER_H__
#define __DATA_RECEIVER_H__

#include <zeromq/zmq.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <stdint.h>
#include <meta.pb.h>

#include <time.h>
#ifndef WIN32
#include <pthread.h>
#include <unistd.h>
#include <sys/time.h>
#endif

#include <matrix/Matrix.h>

#include <ros/ros.h>

static inline long long GetTimeStamp() {
#ifdef WIN32
  time_t tt = time(NULL);
  SYSTEMTIME systime;
  GetLocalTime(&systime);

  struct tm *tm_time;
  tm_time = localtime(&tt);

  return tt * 1000LL + systime.wMilliseconds;
#else
  struct timeval curr_time;
  gettimeofday(&curr_time, NULL);
  return ((long long)curr_time.tv_sec * 1000LL + curr_time.tv_usec / 1000LL);
#endif
}


class FrameInfo {
 public:
  Meta::Meta meta;
  std::vector<std::vector<uint8_t> > img_data;
  std::vector<std::vector<int8_t> > parsing_data;
  int64_t ts;   // timestamp for all messages received for current frame
};

class DataReceiver  
{  
private:  
  void * m_context;   
  void * m_requester;
  char   m_endpoint[100]; 
  zmq_msg_t m_recv_msg;
  
public:  
  DataReceiver();
  ~DataReceiver();

  void init(const char* endpoint);  

  void reconnect();

  int RecvFrame(FrameInfo *frame, ros::Publisher *matrix_msg_pub, matrix::Matrix *matrix_msg);
};  
#endif
