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
 
#include "DataReceiver.h"
#include <assert.h>

#include <opencv2/opencv.hpp>

DataReceiver::DataReceiver() {

};

void DataReceiver::init(const char* endpoint) {
  if (m_endpoint != endpoint)
  {
    strcpy(m_endpoint, endpoint);
  }

  m_context = zmq_ctx_new();
  // Socket to talk to server
  m_requester = zmq_socket(m_context, ZMQ_SUB);//
  int rc = zmq_connect(m_requester, m_endpoint);
  //assert(rc == 0);
  int recvhwm = 1000;
  zmq_setsockopt(m_requester, ZMQ_SUBSCRIBE, "", 0);
  zmq_setsockopt(m_requester, ZMQ_RCVHWM, &recvhwm, sizeof(int));
  zmq_msg_init(&m_recv_msg);

}

DataReceiver::~DataReceiver() {
  zmq_msg_close(&m_recv_msg);
  zmq_close(m_requester);
  zmq_ctx_destroy(m_context);
}

void DataReceiver::reconnect() {

  zmq_msg_close(&m_recv_msg);
  zmq_close(m_requester);
  zmq_ctx_destroy(m_context);

  init(m_endpoint);
}

int DataReceiver::RecvFrame(FrameInfo *frame, ros::Publisher *matrix_msg_pub, matrix::Matrix *matrix_msg) {

  int recv_to = 500;   // 500 ms;
  zmq_setsockopt(m_requester, ZMQ_RCVTIMEO, &recv_to, sizeof(int));

  //int len = zmq_msg_recv(&m_recv_msg, m_requester, 0);

  int more;
  size_t more_size = sizeof(more);

  // recv frame structure
  {
    zmq_msg_t msg_proto;
    int rc = zmq_msg_init(&msg_proto);
    assert(rc == 0);

    rc = zmq_msg_recv(&msg_proto, m_requester, 0);
    if (rc == -1) {
      return 0;
    }

    int protolen = zmq_msg_size(&msg_proto);
    unsigned char *protobuf = (unsigned char *)zmq_msg_data(&msg_proto);
    frame->meta.ParseFromArray((void *)protobuf, protolen);
 
    matrix_msg->ProtoMsg.Len = protolen;
    std::vector<unsigned char> buf(protobuf, protobuf + protolen);
    matrix_msg->ProtoMsg.Buf = buf;

    ROS_INFO("matrix_receiver : protobuf len : %d", protolen);
    zmq_msg_close(&msg_proto);
  }

  zmq_getsockopt(m_requester, ZMQ_RCVMORE, &more, &more_size);
  if (!more) {
    return 0;
  }

  int img_count = frame->meta.data().image_size();
  frame->img_data.resize(img_count);

  matrix_msg->ImageCount = img_count; 
  ROS_INFO("matrix_receiver : img_count : %d", img_count);

  // recv frame data
  for (int i = 0; i < img_count; i++) {
    zmq_msg_t msg;
    int rc = zmq_msg_init(&msg);
    assert(rc == 0);
    rc = zmq_msg_recv(&msg, m_requester, 0);
    int dlen = zmq_msg_size(&msg);

    matrix::MatrixImage matrix_image_msg;
    matrix_image_msg.Len = dlen;
    ROS_INFO("matrix_receiver : image index [%d] size : %d", i, dlen);
    unsigned char *imgbuf = (unsigned char *)zmq_msg_data(&msg); 
    std::vector<unsigned char> img_buf(imgbuf, imgbuf + dlen);
    matrix_image_msg.Buf = img_buf;
    matrix_msg->ImageMsg.push_back(matrix_image_msg);

    zmq_msg_close(&msg);
  }

  // recv parsing data
  zmq_getsockopt(m_requester, ZMQ_RCVMORE, &more, &more_size);
  if (more) {
    int parsing_count = frame->meta.data().structure_perception().parsing_size();
    frame->parsing_data.resize(parsing_count);
    matrix_msg->ParsingCount = parsing_count;
    ROS_INFO("matrix_receiver : parsing_count : %d", parsing_count);
    for (int i = 0; i < parsing_count; i++) {
      zmq_msg_t msg;
      int rc = zmq_msg_init(&msg);
      rc = zmq_msg_recv(&msg, m_requester, 0);
      int parsing_len = zmq_msg_size(&msg);

      matrix::MatrixParsing matrix_parsing_msg;
      matrix_parsing_msg.Len = parsing_len;
      ROS_INFO("matrix_receiver : image index [%d] size : %d", i, parsing_len);
      unsigned char *parsingbuf = (unsigned char *)zmq_msg_data(&msg);
      std::vector<unsigned char> buf(parsingbuf, parsingbuf + parsing_len);
      matrix_parsing_msg.Buf = buf;
      matrix_msg->ParsingMsg.push_back(matrix_parsing_msg);

      zmq_msg_close(&msg);
    }
  }

  while (true) {
    zmq_getsockopt(m_requester, ZMQ_RCVMORE, &more, &more_size);
    if (!more) {
      break;
    }

    zmq_msg_t msg;
    int rc = zmq_msg_init(&msg);
    rc = zmq_msg_recv(&msg, m_requester, 0);
    zmq_msg_close(&msg);
  }
  
  matrix_msg_pub->publish(*matrix_msg);
  frame->ts = GetTimeStamp();

  matrix_msg->ImageMsg.clear();
  matrix_msg->ParsingMsg.clear();
  matrix_msg->ImageMsg.shrink_to_fit();
  matrix_msg->ParsingMsg.shrink_to_fit();  

  
  return 1;
}
