/**
 * @file matrix reveiver.cpp
 * @brief receive message from matrix and publish
 * @author Meng Zhiming
 * @email zhiming.meng@horizon.ai
 * @version 1.0
 */
#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include <list>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <thread>

#include <stdint.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <matrix/Matrix.h>

#include "DataReceiver.h"

// thread for receive
static bool g_bStopReceive = false;
static bool g_bReceiveOver = false;

std::mutex g_lckReceiveOver;
std::condition_variable g_condReceiveOver;

//std::mutex g_lckProcess;
std::condition_variable g_condProcess;
//std::list<std::shared_ptr<FrameInfo> > g_listFrames;
//static const int kMaxListFrameSize = 5;
static bool g_bProcessOver = false;

/**
 * @brief receive thread fucntion
 * @details receive frame data from data provider and add to frame list
 * @param ip_param IP address and port number
 */
void * ThreadRecieve(void *ip_param) {
  
  std::string server = (char *)ip_param;
  ROS_INFO("matrix_receiver: connect to %s", server.c_str());
  
  DataReceiver mq_sub;
  std::string end_point;
  if (server.substr(0, 3) != "tcp") {
    end_point = "tcp://";
    end_point += server;
  }
  else {
    end_point = server;
  }
  mq_sub.init(end_point.c_str());

  unsigned int mq_count = 0;

  float avg_fps = 12.0f;
  float avg_delay = 180.0f;

  bool bRecvFail = false;
  int nRecvFailCount = 0;
  int64_t last_ts = 0;

  // matrix messages publisher
  ros::NodeHandle matrix_nh;
  ros::Publisher matrix_msg_pub = matrix_nh.advertise<matrix::Matrix>("matrix_msg",1);
  matrix::Matrix matrix_msg;

  matrix_msg.header.frame_id = "matrix_frame";

  while (!g_bStopReceive)
  {
    //matrix_msg.header.stamp = ros::Time::now();
    //matrix_msg.header.seq++;

    std::shared_ptr<FrameInfo> frame = std::make_shared<FrameInfo>();

    if (!mq_sub.RecvFrame(frame.get(), &matrix_msg_pub, &matrix_msg)) {
      ROS_INFO("matrix_receiver: Recv Frame Timeout");      
      {
        bRecvFail = true;
        nRecvFailCount++;

        if (nRecvFailCount > 20) {
          mq_sub.reconnect();
          nRecvFailCount = 0;
        }
      }
      continue;
    }
    
    int ts = GetTimeStamp();

#if 0
    {
      std::unique_lock<std::mutex> lck(g_lckProcess);
      if (g_listFrames.size() >= kMaxListFrameSize) {
        g_listFrames.pop_front();
      }
      g_listFrames.push_back(frame);
    }
#endif
    bRecvFail = false;
    nRecvFailCount = 0;
    mq_count++;

    if (last_ts != 0) {
      int64_t time = ts - last_ts;
      if (time > 0) {
        avg_fps = avg_fps * 0.9f + 1000.0f / time * 0.1f;
      }

      int64_t delay = frame->ts - frame->meta.data().image(0).time_stamp();
      avg_delay = avg_delay * 0.9f + delay * 0.1f;

      if (mq_count % 20 == 0) {
        ROS_INFO("matrixcx_receiver: current eth recv: avg fps: %.1f, avg delay: %.1f\n",
              avg_fps, avg_delay);
      }
    }
    last_ts = ts;
  } // end of while

  g_bReceiveOver = true;

  //std::unique_lock<std::mutex> lck(g_lckReceiveOver);
  g_condReceiveOver.notify_all();

  return 0;
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "matrix_receiver");

  /* Set Matrix IP and Port */
  char server[] = "192.168.1.10:5560";

  std::thread th_recv(ThreadRecieve, (void *)server);  
 
  {
    std::unique_lock<std::mutex> lck(g_lckReceiveOver);
    while ((!g_bReceiveOver) && ros::ok) {
      g_condReceiveOver.wait_for(lck,
                                 std::chrono::milliseconds(10));
      
      char c;
      std::cin >> c;
      if (c == 'q') {
        break;
      }
    }
  }

  g_bStopReceive = true;

  if (th_recv.joinable()) {
    th_recv.join();
  }

  google::protobuf::ShutdownProtobufLibrary();

  return 0;
}
