#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <list>
#include <mutex>

#include <condition_variable>
#include <memory>
#include <thread>

#include <stdint.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <matrix/MatrixLanes.h>
#include <matrix/Cameras.h>
#include <matrix/TrafficSigns.h>

#include <matrix/Matrix.h>
#include <matrix/TrafficLights.h>
#include <matrix/MatrixObjects.h>
#include <meta.pb.h>

#include <opencv2/opencv.hpp>
#include <turbojpeg.h>

/**
 * @brief get current system time
 */
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

int tjpeg_decompress(unsigned char *jpg_buffer, int buffer_len, cv::Mat &mat) {
  tjhandle handle = NULL;
  int img_width;
  int img_height;
  int img_subsamp;
  int img_colorspace;
  int flags = 0;
  int tjpf_flags = TJPF_BGR;
  int def_w = 1280;
  int def_h = 720;
  int scale = 1;

  handle = tjInitDecompress();
  if (NULL == handle) {
    return -1;
  }

  int ret = tjDecompressHeader3(handle, jpg_buffer, buffer_len, &img_width, &img_height, &img_subsamp, &img_colorspace);
  if (0 != ret) {
    tjDestroy(handle);
    return -1;
  }

  std::cout << "jpeg width : " << img_width << std::endl;
  std::cout << "jpeg height : " << img_height << std::endl;
  std::cout << "jpeg subsamp : " << img_subsamp << std::endl;

  int dst_w = img_width / scale;
  int dst_h = img_height / scale;
  
  if (tjpf_flags == TJPF_BGR) {
    mat.create(dst_h, dst_w, CV_8UC3);
  } else {
    mat.create(dst_h, dst_w, CV_8UC1);
  }

  ret = tjDecompress2(handle, jpg_buffer, buffer_len,
    mat.data, mat.cols, mat.step, mat.rows,
    tjpf_flags, TJFLAG_FASTUPSAMPLE | TJFLAG_NOREALLOC);
  if (ret != 0) {
    tjDestroy(handle);
    return -1;
  }

  tjDestroy(handle);
  return 0;
}


ros::Publisher camera_pub_custom;
ros::Publisher lines_pub_custom[4];
ros::Publisher object_pub_custom[4];
ros::Publisher trafficlight_pub_custom[4];
ros::Publisher trafficsign_pub_cutom[4];
image_transport::Publisher image_pub_custom[4];
image_transport::Publisher fullimage_pub_custom[4];
image_transport::Publisher onechannelimage_pub_custom[4];
image_transport::Publisher channelsimage_pub_custom[4];
image_transport::Publisher laneimage_pub_custom[4];

bool is_pub_line;
bool is_pub_object;
bool is_pub_light;
bool is_pub_trafficsign;
bool is_pub_fullimage;
bool is_pub_onechannelimage;
bool is_pub_channelsimage;
bool is_pub_laneimage;
bool is_pub_image;


/**
 * @brief class UndistortInfo
 * Used to get undistort imge from raw image
 */
class UndistortInfo {
public:
  UndistortInfo() {
    inited = false;
  }
  void InitUndistort(int w, int h, int type,
                     float focal_u, float focal_v,
                     float center_u, float center_v,
                     const float *distort_param,
                     int distort_param_num);

  cv::Mat UndistortImage(cv::Mat image);

  int img_w_;
  int img_h_;
  float focal_u_;
  float focal_v_;
  float center_u_;
  float center_v_;
  std::vector<float> distort_;
  std::vector<cv::Mat> distort_map1_;
  std::vector<cv::Mat> distort_map2_;
  bool inited;
};

/**
* @brief class ParsingInfo
* Used to get parisng label image
*/
class ParsingInfo {
public:
    ParsingInfo() {
        inited = false;
    }

    void InitParsingInfo(const CommonProto::ParsingSetting *parsing_setting,
                         const CommonProto::Image *parsing,
                         const CommonProto::Image *image);

    cv::Mat GetParsingLabelMat(const uint8_t *parsing_data,
                            int len);

    cv::Mat GetParsingMat(cv::Mat &label);

    cv::Mat GetParsingMatByName(cv::Mat &label,std::string name);

    cv::Mat GetParsingMatByNames(cv::Mat &label,std::initializer_list<std::string> names);

    int parsing_label_width_;
    int parsing_label_height_;

    int parsing_format_;
    int parsing_type_;
    int parsing_label_channels_;

    int parsing_width_;
    int parsing_height_;

    std::vector<uint64> channel_label_;
    std::vector<CommonProto::Color> channel_color_;

    std::vector<uint64> boundspace_channels_;
    std::vector<uint64> freespace_channels_;

    std::map<std::string, uint64> label_map_;

    cv::Mat parsing_label_mat_;

    int parsing_id_;
    bool parsing_id_inited_;


    std::set<int> parsing_ids_;
    bool parsing_ids_inited_;

    bool inited;
};

enum ParsingType {
    ParsingType_Parsing = 0,
    ParsingType_LaneParsing = 1,
};
static int parsing_label_count = 42;
const std::string parsing_label_name[42] = {
        "Road",
        "Sidewalk",
        "Building",
        "LaneMarking",
        "Fence",
        "Pole",
        "TrafficLight",
        "TrafficSign",
        "Vegetation",
        "Terrain",
        "Sky",
        "Person",
        "Rider",
        "Car",
        "Truck",
        "Bus",
        "Train",
        "Motorcycle",
        "Bicycle",
        "GuidePost",
        "CrosswalkLine",
        "TrafficArrow",
        "SignLine",
        "GuideLine",
        "TrafficCone",
        "Barrel",
        "StopLine",
        "SpeedMark",
        "Diamond",
        "BicycleSign",
        "SpeedBumps",
        "Tricycle",
        "SlowDownTriangle",
        "Mask",
        "PotHole",
        "Other",
        "Curb",
        "ParkingLine",
        "ParkingSpace",
        "ParkingRod",
        "ParkingLock",
        "Column",
};

/**
* @brief initialize the parsing settings
* @param parsing_setting parsing setttings from the meta data
* @param parsing parsing label image information from the meta data
* @image image origin image information from the meta data
* @return pointer to result parsing label buffer
*/
void ParsingInfo::InitParsingInfo(const CommonProto::ParsingSetting *parsing_setting,
                                  const CommonProto::Image *parsing,
                                  const CommonProto::Image *image) {
    if (inited) {
        return;
    }

    parsing_label_width_ = parsing->width();
    parsing_label_height_ = parsing->height();
    parsing_format_ = parsing->format();

    parsing_width_ = image->width();
    parsing_height_ = image->height() / 8 / 8 * 8 * 8;

    parsing_label_channels_ = parsing_setting->parsing_channels();
    parsing_type_=parsing_setting->parsing_type();

    if (channel_label_.size() < parsing_label_channels_) {
        channel_label_.resize(parsing_label_channels_);
    }

    if (channel_color_.size() < parsing_label_channels_) {
        channel_color_.resize(parsing_label_channels_);
    }

    for (int idx = 0; idx < parsing_label_channels_; idx++) {
        channel_label_[idx] = parsing_setting->channel_labels(idx);

        channel_color_[idx].set_b(parsing_setting->channel_colors(idx).b());
        channel_color_[idx].set_g(parsing_setting->channel_colors(idx).g());
        channel_color_[idx].set_r(parsing_setting->channel_colors(idx).r());
    }

    for (int idx_type = 0; idx_type < parsing_label_count; idx_type++) {
        label_map_.insert(std::pair<std::string, long long>(parsing_label_name[idx_type], 1LL << idx_type));
    }

    inited = true;

    parsing_ids_inited_ = false;
    parsing_id_inited_ = false;
}

/**
* @brief get the full parsing image
* @details this functuion takes the color image of the
*          parsing label from the label mat
* @param none
* @return pointer to result parsing label buffer
*/
cv::Mat ParsingInfo::GetParsingMat(cv::Mat &label) {
    if (label.empty()) {
        return cv::Mat();
    }

    cv::Mat color_label(parsing_label_height_, parsing_label_width_, CV_8UC3);

    for (int y = 0; y < parsing_label_height_; y++) {
        uint8_t *pl = (uint8_t *) label.ptr(y);
        uint8_t *color = (uint8_t *) color_label.ptr(y);
        for (int x = 0; x < parsing_label_width_; x++) {
            int max_id = pl[x];
            if (parsing_type_ == ParsingType_LaneParsing) {
                for (int idx = 0; idx < parsing_label_channels_; idx++) {
                    if (pl[x] & channel_label_[idx]) {
                        max_id = idx;
                        break;
                    }
                }
            }
            *color++ = channel_color_[max_id].r();
            *color++ = channel_color_[max_id].g();
            *color++ = channel_color_[max_id].b();
        }
    }

    return color_label;
}

/**
* @brief get one channel parsing image
* @details this functuion takes the color image of the channel @name
* @param name channel name of the label
*        see @parsing_label_name
* @return pointer to result parsing label buffer
*/
cv::Mat ParsingInfo::GetParsingMatByName(cv::Mat &label,std::string name) {
    if (label.empty()) {
        return cv::Mat();
    }

    cv::Mat color_label(parsing_label_height_, parsing_label_width_, CV_8UC3);

    if (!parsing_id_inited_) {
        uint64 parsing_lable;
        parsing_lable = label_map_[name];

        for (int idx = 0; idx < parsing_label_channels_; idx++) {
            if (parsing_lable & channel_label_[idx]) {
                parsing_id_ = idx;
                break;
            }
        }
        parsing_id_inited_ = true;
    }

    for (int y = 0; y < parsing_label_height_; y++) {
        uint8_t *pl = (uint8_t *)label.ptr(y);
        uint8_t *color = (uint8_t *)color_label.ptr(y);
        for (int x = 0; x < parsing_label_width_; x++) {
            int max_id = pl[x];
            if (max_id == parsing_id_) {
                *color++ = channel_color_[max_id].r();
                *color++ = channel_color_[max_id].g();
                *color++ = channel_color_[max_id].b();
            }
            else {
                *color++ = 0x00;
                *color++ = 0x00;
                *color++ = 0x00;
            }
        }
    }

    return color_label;
}

static bool IsNamesParsing(int id, std::set<int> &ids) {
    std::set<int>::iterator it;

    for (it = ids.begin(); it != ids.end(); it++) {
        if ((*it) == id) {
            return true;
        }
    }
    return false;
}

/**
* @brief get the channels of parsing image
* @details this functuion takes the color image of channels @names
* @param namea channel namea of the label
*        see @parsing_label_name
* @return pointer to result parsing label buffer
*/
cv::Mat ParsingInfo::GetParsingMatByNames(cv::Mat &label,std::initializer_list<std::string> names) {
    if (label.empty()) {
        return cv::Mat();
    }

    cv::Mat color_label(parsing_label_height_, parsing_label_width_, CV_8UC3);

    if (!parsing_ids_inited_) {
        std::vector<uint64> parsing_label;
        parsing_label.resize(names.size());
        std::cout << names.size() << " types" << std::endl;

        int parsing_label_type = 0;
        for (auto it = names.begin(); it != names.end(); it++) {
            parsing_label[parsing_label_type] = label_map_[*it];
            std::cout << " parsing label type " << parsing_label_type << " -> "
                      << parsing_label[parsing_label_type] << std::endl;
            parsing_label_type++;
        }

        for (int idx_type = 0; idx_type < parsing_label_type; idx_type++) {
            for (int idx_channel = 0; idx_channel < parsing_label_channels_; idx_channel++) {
                if (parsing_label[idx_type] & channel_label_[idx_channel]) {
                    parsing_ids_.insert(idx_channel);
                    break;
                }
            }
        }
        std::cout << "id types: " << parsing_ids_.size() << std::endl;
        parsing_ids_inited_ = true;
    }

    for (int y = 0; y < parsing_label_height_; y++) {
        uint8_t *pl = (uint8_t *)label.ptr(y);
        uint8_t *color = (uint8_t *)color_label.ptr(y);
        for (int x = 0; x < parsing_label_width_; x++) {
            int max_id = pl[x];
            if(IsNamesParsing(max_id, parsing_ids_)) {
                *color++ = channel_color_[max_id].r();
                *color++ = channel_color_[max_id].g();
                *color++ = channel_color_[max_id].b();
            }
            else {
                *color++ = 0x00;
                *color++ = 0x00;
                *color++ = 0x00;
            }
        }
    }

    return color_label;
}

enum LogEnable {
    LogEnable_CANFrameRaw = 0,
    LogEnable_CANFrame = 1,
    LogEnable_OdometryFrame = 2,
    LogEnable_ObstacleRaw = 3,
    LogEnable_Obstacle = 4,
    LogEnable_FreeSpace = 5,
    LogEnable_Line = 6,
    LogEnable_ParsingSetting = 7,
    LogEnable_Count = 8
};

static int log_enable[LogEnable_Count] = {
        0, //CANFrameRaw
        0, //CANFrame
        0, //OdometryFrame
        0, //ObstacleRaw
        0, //Obstacle
        0, //FreeSpace
        0, //Line
        0, //ParsingSetting
};


static std::vector<UndistortInfo> g_undistort;
static std::vector<uint8_t> g_rledecompressbuf;

static std::vector<ParsingInfo> g_parsinginfo;

void GetParsingInfo(const Meta::Meta *meta);
static bool g_getparsinginfo = false;

const int imageWidth = 1280;
const int imageHeight = 720;

/**
 * @brief get the undistort info
 * @details get undistort parameters from meta and populate
 *          the undistort map of UndistortInfo
 * @param meta pointer to received meta
 */
void GetUndistortInfo(Meta::Meta *meta) {
  int cam_count = meta->data().camera_size();
  if (g_undistort.size() < cam_count) {
    g_undistort.resize(cam_count);
  }
  matrix::Cameras camera;
  camera.header.stamp=ros::Time::now();
  camera.header.frame_id="Cameraparam";
  camera.cameras.resize(cam_count);
  for (int idx = 0; idx < cam_count; idx++) {
    const CommonProto::CameraMatrix &cameramatrix = meta->data().camera_matrix(idx);
    const CommonProto::CameraParam &cam = meta->data().camera(idx);
   // const CommonProto::VCSParam &vcsparam=meta->data()
    camera.cameras[idx].distort.resize(cam.distort().param_size());
    camera.cameras[idx].img2vcsgnd.resize(cameramatrix.mat_img2vcsgnd().size());
    camera.cameras[idx].vcsgnd2img.resize(cameramatrix.mat_vcsgnd2img().size());
    camera.cameras[idx].Vcsparam.resize(6);
    camera.cameras[idx].camera_id=idx;
    camera.cameras[idx].width=imageWidth;
    camera.cameras[idx].height=imageHeight;
    camera.cameras[idx].focal_u=cam.focal_u();
    camera.cameras[idx].focal_v=cam.focal_v();
    camera.cameras[idx].center_u=cam.center_u();
    camera.cameras[idx].center_v=cam.center_v();
    camera.cameras[idx].type=cam.type();
    for(int i=0;i<cam.vcs().translation().size();i++){
       camera.cameras[idx].Vcsparam[i]=cam.vcs().translation(i);
    }
    for(int i=0;i<cam.vcs().rotation().size();i++){
        camera.cameras[idx].Vcsparam[3+i]=cam.vcs().rotation(i);
    }
    for(int i=0;i<cameramatrix.mat_vcsgnd2img().size();i++){
       camera.cameras[idx].vcsgnd2img[i]=cameramatrix.mat_vcsgnd2img(i);
       camera.cameras[idx].img2vcsgnd[i]=cameramatrix.mat_img2vcsgnd(i);
    }
    if (cam.has_distort()) {
        g_undistort[idx].InitUndistort(imageWidth, imageHeight, cam.type(),
                                     cam.focal_u(), cam.focal_v(),
                                     cam.center_u(), cam.center_v(),
                                     cam.distort().param().data(),
                                     cam.distort().param_size());
    }
    for(int i=0;i<g_undistort[idx].distort_.size();i++){
        camera.cameras[idx].distort[i]=g_undistort[idx].distort_[i];
    }
  }
  camera_pub_custom.publish(camera);
}

/**
 * @brief init Undistort
 * @details get undistort map of different scale, avoid re-calculation if input
 *          parameters are quite close to previous ones
 */
void UndistortInfo::InitUndistort(int w, int h, int type,
                                  float focal_u, float focal_v,
                                  float center_u, float center_v,
                                  const float *distort,
                                  int distort_param_num) {
  if (distort_param_num != 4) {
    distort_map1_.clear();
    distort_map2_.clear();
    inited = false;
    return;
  } else if (inited && w == img_w_ && h == img_h_
             && fabs(focal_u - focal_u_) < 0.00001f
             && fabs(focal_v - focal_v_) < 0.00001f
             && fabs(center_u - center_u_) < 0.00001f
             && fabs(center_v - center_v_) < 0.00001f
             && fabs(distort[0] - distort_[0]) < 0.00001f
             && fabs(distort[1] - distort_[1]) < 0.00001f
             && fabs(distort[2] - distort_[2]) < 0.00001f
             && fabs(distort[3] - distort_[3]) < 0.00001f) {
    return;
  }

  img_w_ = w;
  img_h_ = h;
  focal_u_ = focal_u;
  focal_v_ = focal_v;
  center_u_ = center_u;
  center_v_ = center_v;
  distort_.resize(distort_param_num);
  for (int i = 0; i < distort_param_num; i++) {
    distort_[i] = distort[i];
  }

  cv::Mat k_ = (cv::Mat_<float>(3, 3) << focal_u, 0.0f, center_u,
                0.0f, focal_v, center_v,
                0.0f, 0.0f, 1.0f);

  cv::Mat d_(4, 1, CV_32FC1);
  float *dist_data = reinterpret_cast<float *>(d_.ptr(0));
  for (size_t i = 0; i < distort_param_num; i++) {
    dist_data[i] = distort[i];
  }

  if (distort_map1_.size() < 4) {
    distort_map1_.resize(4);
    distort_map2_.resize(4);
  }

  if (type == CommonProto::CameraType::CameraType_FishEye) {
    cv::fisheye::initUndistortRectifyMap(k_, d_,
                                         cv::Matx33d::eye(), k_,
                                         cv::Size(w, h), CV_16SC2,
                                         distort_map1_[0], distort_map2_[0]);
  } else {
    cv::initUndistortRectifyMap(k_, d_,
                                cv::Matx33d::eye(), k_,
                                cv::Size(w, h), CV_16SC2,
                                distort_map1_[0], distort_map2_[0]);
  }

  for (int i = 1; i < 4; i++) {
    int scale = (1 << i);

    cv::resize(distort_map1_[0], distort_map1_[i],
               cv::Size(w / scale, h / scale),
               0.0, 0.0, cv::INTER_NEAREST);
    cv::resize(distort_map2_[0], distort_map2_[i],
               cv::Size(w / scale, h / scale),
               0.0, 0.0, cv::INTER_NEAREST);
    distort_map1_[i] /= scale;
  }

  inited = true;
}

/**
 * @brief Undistort the image
 * @details use pre-generated map to undistort input raw image
 * @param image input raw image
 * @return undistorted image
 */
cv::Mat UndistortInfo::UndistortImage(cv::Mat image) {
  cv::Mat undistort_im;
  if (distort_map1_.empty()) {
    return undistort_im;
  }

 
  int idx = distort_map1_[0].cols / image.cols - 1;

  cv::Mat map1(image.rows, image.cols,
               distort_map1_[idx].type(),
               distort_map1_[idx].data, distort_map1_[idx].step);
  cv::Mat map2(image.rows, image.cols,
               distort_map2_[idx].type(),
               distort_map2_[idx].data, distort_map2_[idx].step);

  cv::Mat mat_undist;
  {
    cv::remap(image, undistort_im,
              map1, map2,
              cv::INTER_LINEAR, cv::BORDER_CONSTANT);
  }

  return undistort_im;
}

enum SubSampleMode {
  SubSample_None = 0,
  SubSample_Invalid = 1,    // reserved
  SubSample_Half = 2,
  SubSample_Quarter = 3,
  SubSample_NoImage = 4,    // 
};

const char *g_image_format[] = {
  "Gray",
  "YV12",
  "JPEG",     // jpeg compressed
  "PNG",      // png compressed
  "CR12",     // reserved
  "BAD",      // reserved
  "NV12",
  "NV21",
  "Timeout"   // timeout image
  "bypass"
};

/**
 * @brief raw image data processing
 * @details parse the raw image from received frame data according to
 *          color mode and do the undistortion
 * @param pointer to frame
 */
bool ParseImage(const matrix::Matrix::ConstPtr &msg, Meta::Meta *meta) {
  static int subsample_rate[] = {
    1, 1, 2, 4, 1
  };

  std::cout << "Image Count :" << msg->ImageCount << std::endl;
  for (int i = 0; i < msg->ImageCount; i++) {
    const CommonProto::Image &img_info = meta->data().image(i);
    uint32_t width = img_info.width();
    uint32_t height = img_info.height();
    int sub_sample = img_info.send_mode();
    int scale = subsample_rate[sub_sample];
    int color_mode = img_info.format();   // for matrix, NV12 and jpeg only

    std::cout << "Image " << i << ": ("
      << width << ", "
      << height << ", "
      << scale << ", "
      << g_image_format[color_mode] << ")" << std::endl;

    cv::Mat image;
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    std::cout << "msg->ImageMsg[" <<i << "].Len:" << msg->ImageMsg[i].Len << std::endl;
    uint8_t *img_data = new unsigned char[msg->ImageMsg[i].Len];
    memcpy(img_data, &msg->ImageMsg[i].Buf[0], msg->ImageMsg[i].Len);
    
    if (color_mode == CommonProto::ImageFormat::TIMEOUT) {
      // this is a fake image when camera interrupt timeout happened
    }
    else if (color_mode == CommonProto::ImageFormat::JPEG) {
      // sample to get jpeg image form raw data
      tjpeg_decompress(img_data, msg->ImageMsg[i].Len, image);
      
    } else {
      uint8_t *img_y = img_data;
      uint8_t *img_uv = img_data + width * height;  // uvuvuv pattern
    
      // sample for a gray image
      cv::Mat mat_gray(height, width, CV_8UC1, img_data, width);

      // sample for resize back to origin image size
      cv::Mat mat_resize(height * scale, width * scale, mat_gray.type());
      cv::resize(mat_gray, mat_resize, mat_resize.size());
      image = mat_resize;
      cv_ptr->image=mat_resize;
    }
      if(i==0){
         // ros_image_pub.publish(cv_ptr->toImageMsg());
      }

    // Get Undistort Image
    cv::Mat undistort_image = g_undistort[i].UndistortImage(image);
    if(is_pub_image){
      sensor_msgs::ImagePtr msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", undistort_image).toImageMsg();
      image_pub_custom[i].publish(msg_image);
    }

    delete[] img_data;
    img_data = NULL;

  }
  return true;
 
}


/**
 * @brief meta parsing
 * @details parse interested info from meta, like sensor data, odo info,
 *          obstacle info, freespace info ... and do further processing
 *          if needed, please refer to corresponding proto files for message
 *          details
 * @param meta pointer to meta
 */
bool ParseMeta(Meta::Meta *meta) {
  std::cout << "--------------------------------NEW META--------------------------------" << std::endl;

  int img_count = meta->data().image_size();

  int frame_id_ = meta->frame_id();
  std::cout << "Frame id: " << frame_id_ << std::endl;

  if (meta->data().has_sensor()) {
    const MetaData::SensorFrame &sensor_frame = meta->data().sensor();

    if (sensor_frame.can_frames_raw_size()) {
      for (int i = 0; i < sensor_frame.can_frames_raw_size(); i++) {
        const CANProto::CANFrameRaw &raw_proto = sensor_frame.can_frames_raw(i);
        // TODO: can raw data
      }
    }
  }

  if (meta->data().has_odometry()) {
    const CommonProto::OdometryFrame &odo_frame = meta->data().odometry();
    // TODO: odometry info
  }
  /* Perception Info */
  const MetaData::StructurePerception &percepts = meta->data().structure_perception();
  for (int idx = 0; idx < img_count; idx++) {
    std::cout << "Camera " << idx << ": " << std::endl;

    if (idx >= percepts.obstacles_raws_size()) {
      continue;
    }
    const CommonProto::Image &image = meta->data().image(idx);
    int width = image.width();
    int height = image.height();
    long long time_stamp = image.time_stamp();

	/* Obstacle Raw info */
    const CommonProto::ObstacleRaws &obs_raws = percepts.obstacles_raws(idx);
    const float raw_conf_scale = obs_raws.conf_scale();
    for (int i = 0; i < obs_raws.obstacle_size(); i++) {
      const CommonProto::ObstacleRaw &obs_raw = obs_raws.obstacle(i);
      float conf = obs_raw.conf() * raw_conf_scale;

      if (obs_raw.model() == CommonProto::ObstacleRawModel_Car) {
        // vehicle rear/head, reserved
        const CommonProto::Rect &rect = obs_raw.rect();
      } else if (obs_raw.model() == CommonProto::ObstacleRawModel_FullCar) {
        // vehicle full
        const CommonProto::Rect &rect = obs_raw.rect();
      } else if (obs_raw.model() == CommonProto::ObstacleRawModel_Ped) {
        // ped
        const CommonProto::Rect &rect = obs_raw.rect();
      } else if (obs_raw.model() == CommonProto::ObstacleRawModel_TrafficSign) {
        // traffic sign
        const CommonProto::Rect &rect = obs_raw.rect();
      } else if (obs_raw.model() == CommonProto::ObstacleRawModel_TrafficLight) {
        // traffic light
        const CommonProto::Rect &rect = obs_raw.rect();
      } //else if (obs_raw.model() == CommonProto::ObstacleRawModel_Cyclist) {
        // cyclist
        //const CommonProto::Rect &rect = obs_raw.rect();
      //}
    }


	/* Obstacle Info */
	const CommonProto::Obstacles &obstacles = percepts.obstacles(idx);
	matrix::MatrixObjects objects;
    matrix::TrafficLights lights_;
    matrix::TrafficSigns signs;
	objects.header.stamp=ros::Time::now();
	objects.header.frame_id="matrix_obiect";
	lights_.header.stamp=ros::Time::now();
	lights_.header.frame_id="matrix_light";
	signs.header.stamp=ros::Time::now();
	signs.header.frame_id="matrix_trafficSign";
    lights_.lights.resize(obstacles.obstacle_size());
    objects.objects.resize(obstacles.obstacle_size());
    signs.trafficSign.resize(obstacles.obstacle_size());
    const float conf_scale = obstacles.conf_scale();
	for (int i = 0; i < obstacles.obstacle_size(); i++) {
		const CommonProto::Obstacle &obs = obstacles.obstacle(i);
		float conf = obs.conf() * conf_scale;

		objects.objects[i].header.stamp=ros::Time::now();
		// Obstacle ID
		std::cout << "The Obstacle ID is : " << obs.id() << std::endl;
		objects.objects[i].ObsId= obs.id();
		// Obstacle Type
		CommonProto::ObstacleType type = (CommonProto::ObstacleType) obs.type();
		objects.objects[i].type=ObstacleType_Name(type);

		std::cout << "The Obstacle type is : " << ObstacleType_Name(type) << std::endl;
		// Obstacle Confidence
		std::cout << "The Obstacle confidence is : " << conf << std::endl;
		// Obstacle Life Time
		std::cout << "The Obstacle life time is : " << obs.life_time() << std::endl;
		// Obstacle Age
		std::cout << "The Obstacle age is : " << obs.age() << std::endl;

		/* img info */
		const CommonProto::ImageSpaceInfo &img_info = obs.img_info();
		if (obs.has_img_info()) {
			if (img_info.has_rect()) {
			    objects.objects[i].position_shape.image_bottom=img_info.rect().bottom();
			    objects.objects[i].position_shape.image_top=img_info.rect().top();
			    objects.objects[i].position_shape.image_left=img_info.rect().left();
			    objects.objects[i].position_shape.image_right=img_info.rect().right();
			    std::cout<<"objects.objects[i].position_shape.image_lef"<<objects.objects[i].position_shape.image_left<<std::endl;
				std::cout << "The Obstacle image space : " << "(u1,v1) = (" << img_info.rect().left() << "," << img_info.rect().top() << ")" << "  " << "(u2,v1) = (" << img_info.rect().right() << "," << img_info.rect().top() << ")" << std::endl;
				std::cout << "                           " << "(u1,v2) = (" << img_info.rect().left() << "," << img_info.rect().bottom() << ")" << "  " << "(u2,v2) = (" << img_info.rect().right() << "," << img_info.rect().bottom() << ")" << std::endl;
				// std::cout << "the obstacle (pedestrian) image space is :" << img_info.rect().left() << " " << img_info.rect().top() << " " << img_info.rect().right() << " " << img_info.rect().bottom() << std::endl;
			}
			if (img_info.has_box()) {
				// Print the 3D box (for vehicles)
				objects.objects[i].point_box.resize(8);
				objects.objects[i].point_box[0].x=img_info.box().lower_lt().x();
                objects.objects[i].point_box[0].y=img_info.box().lower_lt().y();
                objects.objects[i].point_box[0].z=img_info.box().lower_lt().z();
                objects.objects[i].point_box[1].x=img_info.box().lower_lb().x();
                objects.objects[i].point_box[1].y=img_info.box().lower_lb().y();
                objects.objects[i].point_box[1].z=img_info.box().lower_lb().z();
                objects.objects[i].point_box[2].x=img_info.box().lower_rb().x();
                objects.objects[i].point_box[2].y=img_info.box().lower_rb().y();
                objects.objects[i].point_box[2].z=img_info.box().lower_rb().z();
                objects.objects[i].point_box[3].x=img_info.box().lower_rt().x();
                objects.objects[i].point_box[3].y=img_info.box().lower_rt().y();
                objects.objects[i].point_box[3].z=img_info.box().lower_rt().z();
                objects.objects[i].point_box[4].x=img_info.box().upper_lt().x();
                objects.objects[i].point_box[4].y=img_info.box().upper_lt().y();
                objects.objects[i].point_box[4].z=img_info.box().upper_lt().z();
                objects.objects[i].point_box[5].x=img_info.box().upper_lb().x();
                objects.objects[i].point_box[5].y=img_info.box().upper_lb().y();
                objects.objects[i].point_box[5].z=img_info.box().upper_lb().z();
                objects.objects[i].point_box[6].x=img_info.box().upper_rb().x();
                objects.objects[i].point_box[6].y=img_info.box().upper_rb().y();
                objects.objects[i].point_box[6].z=img_info.box().upper_rb().z();
                objects.objects[i].point_box[7].x=img_info.box().upper_rt().x();
                objects.objects[i].point_box[7].y=img_info.box().upper_rt().y();
                objects.objects[i].point_box[7].z=img_info.box().upper_rt().z();
                std::cout << "The Obstacle 3D box : " << std::endl;
				std::cout << "                      " << "lower_lt point is   : " << "(x,y,z) = " << " (" << img_info.box().lower_lt().x() << "," << img_info.box().lower_lt().y() << "," << img_info.box().lower_lt().z() << ")" << std::endl;
				std::cout << "                      " << "lower_lb point is   : " << "(x,y,z) = " << " (" << img_info.box().lower_lb().x() << "," << img_info.box().lower_lb().y() << "," << img_info.box().lower_lb().z() << ")" << std::endl;
				std::cout << "                      " << "lower_rb point is   : " << "(x,y,z) = " << " (" << img_info.box().lower_rb().x() << "," << img_info.box().lower_rb().y() << "," << img_info.box().lower_rb().z() << ")" << std::endl;
				std::cout << "                      " << "lower_rt point is   : " << "(x,y,z) = " << " (" << img_info.box().lower_rt().x() << "," << img_info.box().lower_rt().y() << "," << img_info.box().lower_rt().z() << ")" << std::endl;
				std::cout << "                      " << "upper_lt point is   : " << "(x,y,z) = " << " (" << img_info.box().upper_lt().x() << "," << img_info.box().upper_lt().y() << "," << img_info.box().upper_lt().z() << ")" << std::endl;
				std::cout << "                      " << "upper_lb point is   : " << "(x,y,z) = " << " (" << img_info.box().upper_lb().x() << "," << img_info.box().upper_lb().y() << "," << img_info.box().upper_lb().z() << ")" << std::endl;
				std::cout << "                      " << "upper_rb point is   : " << "(x,y,z) = " << " (" << img_info.box().upper_rb().x() << "," << img_info.box().upper_rb().y() << "," << img_info.box().upper_rb().z() << ")" << std::endl;
				std::cout << "                      " << "upper_rt point is   : " << "(x,y,z) = " << " (" << img_info.box().upper_rt().x() << "," << img_info.box().upper_rt().y() << "," << img_info.box().upper_rt().z() << ")" << std::endl;
			}
		}
		std::cout << std::endl;

		/* world info */
		const CommonProto::WorldSpaceInfo &world_info = obs.world_info();
		if (obs.has_world_info()) {
			if (world_info.has_position()) {
			    objects.objects[i].position_shape.x=world_info.position().x();
			    objects.objects[i].position_shape.y=world_info.position().y();
				std::cout << "The Obstacle position is : " << world_info.position().x() << " " << world_info.position().y();
				if (world_info.position().has_z()) {
				    objects.objects[i].position_shape.z=world_info.position().z();
					std::cout << " " << world_info.position().z() << std::endl;
				}
			}
			if (world_info.has_vel()) {
				if (world_info.vel().has_vz()) {
				    objects.objects[i].position_shape.vel_x=world_info.vel().vx();
				    objects.objects[i].position_shape.vel_y=world_info.vel().vy();
				    objects.objects[i].position_shape.vel_z=world_info.vel().vz();
				    std::cout<<"vel"<<objects.objects[i].position_shape.vel_x<<std::endl;
					std::cout << "The Obstacle velocity is : (x, y, z) = ( " << world_info.vel().vx() << " , " << world_info.vel().vy() << " , " << world_info.vel().vz() << " )" << std::endl;
				}
				else {
                    objects.objects[i].position_shape.vel_x=world_info.vel().vx();
                    objects.objects[i].position_shape.vel_y=world_info.vel().vy();
					std::cout << "The Obstacle velocity is : (x, y) = ( " << world_info.vel().vx() << " , " << world_info.vel().vy() << " )" << std::endl;
				}
			}
			if (world_info.has_yaw()) {
			    objects.objects[i].position_shape.yaw=world_info.yaw();
				std::cout << "The Obstacle yaw is : " << world_info.yaw() << std::endl;
			}
			if (world_info.has_length()) {
			    objects.objects[i].position_shape.height=world_info.length();
				std::cout << "The Obstacle length is : " << world_info.length() << std::endl;
			}
			if (world_info.has_width()) {
			    objects.objects[i].position_shape.width=world_info.width();
				std::cout << "The Obstacle width is : " << world_info.width() << std::endl;
			}
			if (world_info.has_height()) {
			    objects.objects[i].position_shape.height=world_info.height();
				std::cout << "The Obstacle height is : " << world_info.height() << std::endl;
			}
			if (world_info.has_poly()) {
				int polygon_count = world_info.poly().pts_size();
				std::cout << "The Obstacle Polygon is : " << std::endl;
				for (int k = 0; k < polygon_count; k++) {
					if (world_info.poly().pts(k).has_z()) {
						std::cout << "polygon_" << k << " : ( " << world_info.poly().pts(k).x() << " , " << world_info.poly().pts(k).y() << " , " << world_info.poly().pts(k).z() << " )" << std::endl;
					}
					else {
						std::cout << "polygon_" << k << " : ( " << world_info.poly().pts(k).x() << " , " << world_info.poly().pts(k).y() << " )" << std::endl;
					}
				}
			}
			/*							if (world_info.has_traversable()) {
			std::cout << "obstacle traversable is : " << world_info.traversable() << std::endl;
			}
			if (world_info.has_hmw()) {
			std::cout << "obstacle HMW is : " << world_info.hmw() << std::endl;
			}
			if (world_info.has_ttc()) {
			std::cout << "obstacle TTC is : " << world_info.ttc() << std::endl;
			}
			if (world_info.has_curr_lane()) {
			std::cout << "obstacle current lane is : " << world_info.curr_lane() << std::endl;
			}
			*/
		}
		std::cout << std::endl;

		/* Traffic Light Info */
		if (obs.type() == CommonProto::ObstacleType_TrafficLight) {
            lights_.lights[i].ObsId=obs.id();
		    lights_.lights[i].position_shape.image_bottom=img_info.rect().bottom();
		    lights_.lights[i].position_shape.image_top=img_info.rect().top();
		    lights_.lights[i].position_shape.image_left=img_info.rect().left();
		    lights_.lights[i].position_shape.image_right=img_info.rect().right();
            lights_.lights[i].position_shape.height=world_info.height();
            lights_.lights[i].position_shape.width=world_info.width();
            lights_.lights[i].position_shape.length=world_info.length();
		    lights_.lights[i].position_shape.x=world_info.position().x();
		    lights_.lights[i].position_shape.y=world_info.position().y();
		    lights_.lights[i].position_shape.z=world_info.position().z();
		    lights_.lights[i].position_shape.yaw=world_info.yaw();
			for (int k = 0; k < obs.property_size(); k++) {
				/* traffic light style */
				if (k == 0) {
					CommonProto::TrafficLightStyle style = (CommonProto::TrafficLightStyle) obs.property(k);
					if (CommonProto::TrafficLightStyle_IsValid(style))
					{
                        lights_.lights[i].type=CommonProto::TrafficLightStyle_Name(style);
						std::cout << "The Traffic Light Style is : " << CommonProto::TrafficLightStyle_Name(style) << std::endl;
					}
				}
				/* traffic light status */
				if (k == 1) {
					CommonProto::TrafficLightStatus status = (CommonProto::TrafficLightStatus) obs.property(k);
					if (CommonProto::TrafficLightStatus_IsValid(status)) {
					    lights_.lights[i].status=CommonProto::TrafficLightStatus_Name(status);
						std::cout << "The Traffic Light Status is : " << CommonProto::TrafficLightStatus_Name(status) << std::endl;
					}
				}
			}
		}

		/* Traffic Sign Info */
		if (obs.type() == CommonProto::ObstacleType_TrafficSign) {
		    signs.trafficSign[i].ObsId=obs.id();
            signs.trafficSign[i].position_shape.image_right=img_info.rect().right();
            signs.trafficSign[i].position_shape.image_top=img_info.rect().top();
            signs.trafficSign[i].position_shape.image_left=img_info.rect().left();
            signs.trafficSign[i].position_shape.image_right=img_info.rect().right();
            signs.trafficSign[i].position_shape.height=world_info.height();
            signs.trafficSign[i].position_shape.width=world_info.width();
            signs.trafficSign[i].position_shape.length=world_info.length();
            signs.trafficSign[i].position_shape.x=world_info.position().x();
            signs.trafficSign[i].position_shape.y=world_info.position().y();
            signs.trafficSign[i].position_shape.z=world_info.position().z();
            signs.trafficSign[i].position_shape.yaw=world_info.yaw();
			for (int k = 0; k < obs.property_size(); k++) {
				CommonProto::TrafficSignType sign = (CommonProto::TrafficSignType) obs.property(k);
				//signs.trafficSign[i].type=CommonProto::TrafficSignType_Name(sign);
				std::cout << "The Traffice Sign Info is : " << CommonProto::TrafficSignType_Name(sign) << std::endl;
			}
		}
	}   // end for obstacles(i)
	if(is_pub_object)
    object_pub_custom[idx].publish(objects);
	if(is_pub_light)
	trafficlight_pub_custom[idx].publish(lights_);
	if(is_pub_trafficsign)
	trafficsign_pub_cutom[idx].publish(signs);
	std::cout << std::endl;


	/* Lines Info */
	const CommonProto::Lines &lines = percepts.lines(idx);
	int lines_count = percepts.lines_size();
	std::cout << std::endl;
	std::cout << "---------------------------------------------------------" << std::endl;
	if (lines.has_cam_id()) {
		std::cout << "Lines camera ID is : " << lines.cam_id() << std::endl;
	}
	int line_count = lines.lines_size();
    matrix::MatrixLanes lines_;
    lines_.header.stamp=ros::Time::now();
    lines_.header.frame_id="matrix_line";
    lines_.lines.resize(line_count);
	for (int j = 0; j < line_count; j++) {
		CommonProto::Line line = lines.lines(j);
		lines_.lines[j].coefs.resize(4);
		std::cout << std::endl;
		lines_.lines[j].id= line.id();
		std::cout << "Line ID is : " << line.id() << std::endl;
		if (line.has_life_time()) {
		    lines_.lines[j].life_time= line.life_time();
			std::cout << "Line life time is : " << line.life_time() << std::endl;
		}
        std::cout << "Line End Point & Start Point are : " << std::endl;
        for (int k = 0; k < line.end_points_size(); k++) {
            std::cout << "x" << k << " = " << line.end_points(k).x() << std::endl;
        }
        float xe=line.end_points(0).x();
        float xs=line.end_points(1).x();
        float ys=line.coeffs(0);
        float ye=line.coeffs(0);
        lines_.lines[j].start_point.x=xs;
        lines_.lines[j].end_point.x=xe;
		std::cout << "Line coeffs are : " << std::endl;
		for (int k = 0; k < line.coeffs_size(); k++) {
			std::cout << "coeff" << k << " = " << line.coeffs(k) << std::endl;
			lines_.lines[j].coefs[k]= line.coeffs(k);
		}
		for(int k=1;k<line.coeffs().size();k++){
		    ys+=line.coeffs(k)*xs;
		    ye+=line.coeffs(k)*xe;
		    xs*=xs;
		    xe*=xe;
		}
		lines_.lines[j].start_point.y=ys;
		lines_.lines[j].end_point.y=ye;
		if(line.has_dist_to_front_wheel()){
            lines_.lines[j].dist_to_front_wheel=line.dist_to_front_wheel();
            std::cout<<"dist_to_front_wheel"<<line.dist_to_front_wheel()<<std::endl;
		}
		if (line.has_type()) {
			CommonProto::LineType value = (CommonProto::LineType)line.type();
			lines_.lines[j].type= line.type();
			// std::string line_Type = CommonProto::LineType_Name(value);
			std::string line_Model;
			std::string line_Type;
			std::string line_Color;
			std::string line_Info;
/*			// Line Model Info
			if (value & (int)0) { // LINE_RAW
				line_Model = "LINE_RAW";
			}
			else { // LINE_ROAD
				line_Model = "LINE_ROAD";
			}
			std::cout << "Line model is : " << line_Model << std::endl;
*/			// Line Type Info
			if (value & (int)1024) { // LINE_DASH
				line_Type = "LINE_DASH";
			}
			else if (value & (int)2048) { // LINE_SOLID
				line_Type = "LINE_SOLID";
			}
			else if (value & (int)65536) { // LINE_FENCE
				line_Type = "LINE_FENCE";
			}
			else { // NO_TYPE
				line_Type = "NO_TYPE";
			}
			std::cout << "Line type is : " << line_Type << std::endl;
/*			// Line Color Info
			if (value & (int)4096) { // LINE_WHITE
				line_Color = "LINE_WHITE";
			}
			else if (value & (int)9192) { // LINE_YELLOW
				line_Color = "LINE_YELLOW";
			}
			else if (value & (int)16384) { // LINE_BLUE
				line_Color = "LINE_BLUE";
			}
			else { // NO_COLOR
				line_Color = "NO_COLOR";
			}
			std::cout << "Line color is : " << line_Color << std::endl;
			// Road Model Info
			if (value & (int)1) { // LINE_LEFT
				line_Info = "LINE_LEFT";
			}
			else if (value & (int)2) { // LINE_RIGHT
				line_Info = "LINE_RIGHT";
			}
			else if (value & (int)4) { // LINE_LEFT_LEFT
				line_Info = "LINE_LEFT_LEFT";
			}
			else if (value & (int)8) { // LINE_RIGHT_RIGHT
				line_Info = "LINE_RIGHT_RIGHT";
			}
			else { // NO_INFO
				line_Info = "NO_INFO";
			}
			std::cout << "Line info is : " << line_Info << std::endl;
			*/
		}
/*		if (line.has_source()) {
			std::cout << "Line source is : " << line.source() << std::endl;
		}
		if (line.has_dist_to_front_wheel()) {
			std::cout << "Line distance to front wheel is : " << line.dist_to_front_wheel() << std::endl;
		}

*/
        std::cout << std::endl; // empty line
	} // end for lines info
	if(is_pub_line)
    lines_pub_custom[lines.cam_id()].publish(lines_);
	/* Free Space Info */
    const CommonProto::FreeSpacePoints &free_pts = percepts.scan_pts(idx);
    for (int i = 0; i < free_pts.pts_img_size(); i++) {
      int label = free_pts.property(i);
      if (label == CommonProto::FreeSpacePointType::ParsingLabelType_Invalid) {
        continue;
      }
      const CommonProto::Point &pt_img = free_pts.pts_img(i);
      const CommonProto::Point &pt_vcs = free_pts.pts_vcs(i);
      // TODO: deal with free space points
    }  // end for free space points

  }  // end for camera idx
  return true;
}

/**
* @brief get parsing label property information
* @details this function get the parsing settings and
*          the property related with the parsing label
* @param meta pointer to meta
*/
void GetParsingInfo(const Meta::Meta *meta) {
    if (g_getparsinginfo) {
        return;
    }

    int setting_count = meta->data().structure_perception().parsing_settings_size();

    if (g_parsinginfo.size() < setting_count) {
        g_parsinginfo.resize(setting_count);
    }

    for (int idx = 0; idx < setting_count; idx++) {
        if (idx < meta->data().structure_perception().parsing_settings_size()) {
            CommonProto::ParsingSetting parsing_setting =
                    meta->data().structure_perception().parsing_settings(idx);

            //PrintParsingSettings(parsing_setting);

            g_parsinginfo[idx].InitParsingInfo(
                    &meta->data().structure_perception().parsing_settings(idx),
                    &meta->data().structure_perception().parsing(0),
                    &meta->data().image(0));
        }
    }
    g_getparsinginfo = true;
}
/**
 * @brief get parsing label directly from feature map
 * @details this function takes feature map as input and get the channel with
 *          max conf as corresponding label
 * @param parsing parsing info from meta data
 * @param parsing_data parsing data
 * @return pointer to result parsing label buffer
 */
uint8_t * GetParsingLabelDirectly(const CommonProto::Image &parsing,
                                  const uint8_t *parsing_data) {
  // this function support max 32 channel parsing data
  if (parsing.channel() > 32) {
    return NULL;
  }

  int parsing_w = parsing.width();
  int parsing_h = parsing.height();
  int parsing_c = parsing.channel();
  int parsing_depth = 8;
  int parsing_align = 16;
  int channel_step = parsing_depth / 8 * parsing_w * parsing_h * parsing_align;
  int channel_parse_count = (parsing_c + parsing_align - 1) / parsing_align;

  uint8_t *parsing_label = new uint8_t[parsing_w * parsing_h];

  if (channel_parse_count == 1) {
    for (int y = 0; y < parsing_h; y++) {
      const int8_t *ptr = (int8_t *)parsing_data + y * 16 * parsing_w;
      uint8_t *plabel = parsing_label + y * parsing_w;
      for (int x = 0; x < parsing_w; x++) {
        int max_id = 0;
        int max_val = -255;
        const int8_t *conf = ptr + parsing_align - 1;
        for (int i = 0; i < parsing_c; i++) {
          if (*conf > max_val) {
            max_id = i;
            max_val = *conf;
          }
          conf--;
        }
        plabel[x] = max_id;
        ptr += parsing_align;
      }
    }
  } else if (channel_parse_count == 2) {
    for (int y = 0; y < parsing_h; y++) {
      const int8_t *ptr = (int8_t *)parsing_data + y * 16 * parsing_w;
      uint8_t *plabel = parsing_label + y * parsing_w;;
      for (int x = 0; x < parsing_w; x++) {
        int max_id = 0;
        int max_val = -255;
        // first 16 channel
        for (int i = 0; i < parsing_align; i++) {
          if (ptr[x] > max_val) {
            max_id = i;
            max_val = ptr[x];
          }
        }
        // other channels
        const int8_t *conf = ptr + channel_step + parsing_align - 1;
        for (int i = 0; i < parsing_c; i++) {
          if (*conf > max_val) {
            max_id = parsing_align + i;
            max_val = *conf;
          }
          conf--;
        }

        plabel[x] = max_id;
        ptr += parsing_align;
      }
    }
  }

  return parsing_label;
}

/**
 * @brief get parsing label large
 * @details this function will resize input parsing data to a larger size
 * @param parsing parsing info from meta data
 * @param parsing_data parsing data
 * @return parsing label of larger size
 */
cv::Mat GetParsingLabelLarge(const CommonProto::Image &parsing,
                             const int8_t *parsing_data) {
  int im_w = 1280;
  int im_h = 704;

  int parsing_w = parsing.width();
  int parsing_h = parsing.height();
  int parsing_c = parsing.channel();
  int parsing_depth = parsing.depth();
  int parsing_align = parsing.align();

  int channel_step = parsing_depth / 8 * parsing_w * parsing_h * parsing_align;
  int channel_parse_count = (parsing_c + parsing_align - 1) / parsing_align;

  cv::Mat feature = cv::Mat(parsing_h, parsing_w, CV_8UC(parsing_c));
#define ft_type8 int8_t
  {
    feature = cv::Mat(parsing_h, parsing_w, CV_8UC(parsing_c));

    for (int i = 0; i < channel_parse_count; i++) {
      const ft_type8 *src = (ft_type8 *)parsing_data + i * channel_step;
      int ch_base = i * parsing_align;
      int ch_num = std::min<int>(16, parsing_c - i * parsing_align);

      for (int y = 0; y < parsing_h; y++) {
        ft_type8 *dst = (ft_type8 *)feature.ptr(y);
        for (int x = 0; x < parsing_w; x++) {
          ft_type8 *dst_pt = dst + x * parsing_c + ch_base;
          for (int c = 0; c < ch_num; c++) {
            dst_pt[c] = (int)src[parsing_align - 1 - c] + 128;
          }
          src += parsing_align;
        }
      }
    }
  }

  cv::Mat resized_feature;
  cv::resize(feature, resized_feature, cv::Size(im_w, im_h));

  cv::Mat mat_label = cv::Mat(im_h, im_w, CV_8UC1);

  // get max id
  {
    for (int y = 0; y < im_h; y++) {
      uint8_t *data = (uint8_t *)resized_feature.ptr(y);
      uint8_t *label = (uint8_t *)mat_label.ptr(y);
      for (int x = 0; x < im_w; x++) {
        int max_id = -1;
        int max_conf = -1;
        for (int c = 0; c < parsing_c; c++) {
          if (data[c] > max_conf) {
            max_conf = data[c];
            max_id = c;
          }
        }
        label[x] = max_id;

        data += parsing_c;
      }
    }
  }

  return mat_label;
}

/**
 * @brief RLE decompress
 * @details this function do RLE decompress for the input data
 * @param input_data RLE compressed data
 * @param size length of input compressed data
 * @param output_data pointer to output data
 * @param buf_len length of decompressed data
 * @return success or not
 *  @retval 0 success
 *  @retval < 0 failed
 */
static int RLEDecompress(const uint8_t *input_data,
  int size, uint8_t *output_data, int &buf_len) {
  const uint8_t *ptr = input_data;

  if (ptr[0] != 0xFF || ptr[1] != 'R' || ptr[2] != 'L' || ptr[3] != 'E') {
      std::cout << "RLE header mismatch !!!" << std::endl;
      return -1;
  }

  // Skip the magic number first
  ptr += 4;
  // Then the height and width
  uint16_t height = (*ptr++ << 8) & 0xFF00;
  height |= static_cast<uint16_t>(*ptr++);
  uint16_t width = (*ptr++ << 8) & 0xFF00;
  width |= static_cast<uint16_t>(*ptr++);

  if (height <= 0 || width <= 0) {
    return -1;
  }

  if (buf_len < height * width) {
    return -2;
  }

  buf_len = height * width;

  // Then the data
  int rle_cnt = (size - 8) / 3;
  uint8_t *p_im_data = output_data;
  for (int i = 0; i < rle_cnt; ++i) {
    uint8_t label = *ptr++;
    uint16_t cnt = *reinterpret_cast<const uint16_t *>(ptr);
    ptr += 2;

    for (int j = 0; j < cnt; ++j) {
      *p_im_data++ = label;
    }
  }

  return 0;
}

/**
* @brief get parsing label mat
* @details this functuion takes parsing label mat from the buffer
* @param parsing_data parsing data
* @return none
*/
cv::Mat ParsingInfo::GetParsingLabelMat(const uint8_t *parsing_data,
                                     int len) {
    if ((parsing_label_height_ <= 0) || (parsing_label_width_ <= 0)) {
        return cv::Mat();
    }

    if (parsing_format_ != CommonProto::ParsingFormat_Label_RLE) {
        std::cout << "Only RLE is supported only !" << std::endl;
        return cv::Mat();
    }

    int buf_len = parsing_label_width_ * parsing_label_height_;

    static std::vector<uint8_t> buf;
    if (buf.size() < buf_len) {
        buf.resize(buf_len);
    }

    int ret = RLEDecompress((uint8_t *)parsing_data,
                            len, buf.data(), buf_len);
    if (ret != 0) {
        std::cout << "RLE decompress failed !!!" << std::endl;
    }

    return cv::Mat(parsing_label_height_,
                                 parsing_label_width_, CV_8UC1, buf.data(),
                                 parsing_label_width_);
}
/**
 * @brief deal with parsing data
 * @param parsing parsing info from meta data
 * @param parsing_data parsing data
 * @return None
 */
void DealParsingData(const CommonProto::Image &parsing,
                     const uint8_t *parsing_data,
                     int len, int idx) {
  std::cout << "Parsing Size(" << parsing.width() << ", " << parsing.height()
    << ", " << parsing.channel() << ")" << std::endl;

  if (parsing.format() == CommonProto::ParsingFormat_Label) {
    // parsing data is a label image, just use it
    
    // TODO:
    // deal with parsing label
  } else if (parsing.format() == CommonProto::ParsingFormat_Label_RLE) {
    int buf_len = parsing.width() * parsing.height();
    if (g_rledecompressbuf.size() < buf_len) {
      g_rledecompressbuf.resize(buf_len);
    }

    // Notice:
    // this is just a sample code, which runs in a serial mode
    // Please take care of multi-thread safety if running in a concurrent mode
    int ret = RLEDecompress(parsing_data,
        len, g_rledecompressbuf.data(), buf_len);
    if (ret != 0) {
      std::cout << "RLE decompress failed !!!" << std::endl;
    }
    
    // TODO:
    // deal with parsing label further
#if 1
    
    // deal with parsing label further
    // 320*176
    cv::Mat label(parsing.height(), parsing.width(), CV_8UC1, g_rledecompressbuf.data(), parsing.width());

    uint8_t color_list[32][3] = {
      { 128, 64, 128 },
      { 244, 35, 232 },
      { 107, 142, 35 },
      { 152, 251, 152 },
      { 153, 153, 153 },
      { 220, 220, 0 },
      { 250, 170, 30 },
      { 200, 200, 128 },
      { 200, 200, 200 },
      { 220, 20, 60 },
      { 0, 0, 70 },
      { 0, 0, 142 },
      { 70, 70, 70 },
      { 190, 153, 153 },
      { 70, 130, 180 },
      { 0, 64, 64 },
      { 128, 128, 192 },
      { 192, 192, 0 },
      { 64, 192, 0 },
      { 128, 0, 192 },
      { 192, 192, 128 },
      { 255, 0, 0 },
      { 102, 102, 156 },
      { 0, 0, 230 }
    };

    int im_w = 1280;
    int im_h = 704;

    cv::Mat resized_label;
    resized_label = cv::Mat(im_h, im_w, CV_8UC1);
    cv::resize(label, resized_label,
    cv::Size(im_w, im_h), 0.0, 0.0, cv::INTER_NEAREST);

    cv::Mat color_map(im_h, im_w, CV_8UC3);
    for (int y = 0; y < im_h; y++) {
      uint8_t *pl = (uint8_t *)resized_label.ptr(y);
      uint8_t *color = (uint8_t *)color_map.ptr(y);
      for (int x = 0; x < im_w; x++) {
        int max_id = pl[x];
        *color++ = color_list[max_id][2];
        *color++ = color_list[max_id][1];
        *color++ = color_list[max_id][0];
        // *color++ = 0;
        // *color++ = 0;
        // *color++ = 0;
      }
    }

    cv::imwrite("Color_map.png", color_map);

    // Get Undistort label
    cv::Mat undistort_label = g_undistort[idx].UndistortImage(color_map);

    cv::imwrite("Un_map.png", color_map);   
#endif

  } else {
    // parsing data is parsing feature map
    // here is just compare each channel and get max conf channel,
    // you should notice how to deal with aligned data
    // this is the fastest way to get parsing label
    {
      long long st = GetTimeStamp();
      uint8_t *parsing_label = NULL;
      parsing_label = GetParsingLabelDirectly(parsing,
                                              parsing_data);
      long long ed = GetTimeStamp();
      std::cout << "GetParsingLabelDirectly consumes " << (ed - st) << "ms" << std::endl;
      // TODO:
      // deal with parsing label

      // Get Undistort label
      int parsing_w = parsing.width();
      int parsing_h = parsing.height();
      cv::Mat label(parsing_h, parsing_w, CV_8UC1,
                    parsing_label, parsing_w);
      cv::Mat undistort_label = g_undistort[idx].UndistortImage(label);

      if (parsing_label) {
        delete[] parsing_label;
        parsing_label = NULL;
      }
    }

    // P.S. if you want better visual effect,
    //      you need resize each channel to image size,
    //      and then get max conf channel
    {
      long long st = GetTimeStamp();
      cv::Mat parsing_label = GetParsingLabelLarge(parsing,
                                                   (int8_t *)parsing_data);
      long long ed = GetTimeStamp();
      std::cout << "GetParsingLabelLarge consumes " << (ed - st) << "ms" << std::endl;

      // TODO:
      // deal with parsing label

      // Get Undistort label
      cv::Mat undistort_label = g_undistort[idx].UndistortImage(parsing_label);
    }
  }
}


// get ipm matrix
std::vector<cv::Mat> GetIpmMatrix(Meta::Meta *meta, cv::Rect2f roi_,
                                  cv::Size ipm_size_) {
    std::vector<cv::Mat> vec_ipm_matrix;
    MetaData::Data data = meta->data();
    int num = data.camera_matrix_size();
    for (size_t i = 0; i < num; i++) {
        auto cam_matrix = data.camera_matrix(i);
        if (cam_matrix.mat_img2gnd_size() != 9) {
            std::cout << "mat_img2gnd size is not 9" << std::endl;
        }
        cv::Mat img2gnd(3, 3, CV_32FC1);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                img2gnd.at<float>(i, j) = cam_matrix.mat_img2gnd(i * 3 + j);
            }
        }

        cv::Mat T_ = cv::Mat::zeros(3, 3, CV_32FC1);
        T_.at<float>(0, 0) = -roi_.width / ipm_size_.width;
        T_.at<float>(0, 2) = roi_.x + roi_.width;
        T_.at<float>(1, 1) = -roi_.height / ipm_size_.height;
        // T_.at<float>(1, 2) = roi_.y - up_pixel_ * roi_.height / ipm_size_.height;
        T_.at<float>(1, 2) = roi_.y;
        T_.at<float>(2, 2) = 1.0f;

        cv::Mat axiss = (cv::Mat_<float>(3, 3) << 0, 1, 0, 1, 0, 0, 0, 0, 1);
        T_ = axiss * T_;
        cv::Mat M_ = img2gnd * T_;
        // cv::Mat Minv_ = M_.inv();
        vec_ipm_matrix.push_back(M_);
    }
    return vec_ipm_matrix;
}
void matrixCallback(const matrix::Matrix::ConstPtr &msg) {
  ROS_INFO("%d", msg->ProtoMsg.Len);
  
  if (msg->ProtoMsg.Len > 0) {
    unsigned char *buffer = new unsigned char[msg->ProtoMsg.Len];
    if (!msg->ProtoMsg.Buf.empty()) {  
      memcpy(buffer, &msg->ProtoMsg.Buf[0], msg->ProtoMsg.Len);
      Meta::Meta meta;
      meta.ParseFromArray(buffer, msg->ProtoMsg.Len);
      
      ParseMeta(&meta);

      GetUndistortInfo(&meta);

      if(!(meta.data().image(0).send_mode()&4)){
          ParseImage(msg, &meta);
      }


      GetParsingInfo(&meta);
      std::cout << "Parsing Count :" << msg->ParsingCount << std::endl;
      for (int i = 0; i < msg->ParsingCount; i++) {
        
        std::cout << "msg->ParsingMsg[" << i << "].Len:" << msg->ParsingMsg[i].Len << std::endl;
        uint8_t *parsing_data = new unsigned char[msg->ParsingMsg[i].Len];
        memcpy(parsing_data, &msg->ParsingMsg[i].Buf[0], msg->ParsingMsg[i].Len);

#if 0
        DealParsingData(meta.data().structure_perception().parsing(i),
                        parsing_data, msg->ParsingMsg[i].Len, i);
#endif
        cv::Mat label=g_parsinginfo[0].GetParsingLabelMat(parsing_data, msg->ParsingMsg[i].Len);

          // Choose as your needs
        cv::Mat full = g_parsinginfo[0].GetParsingMat(label);
        cv::Mat one_channel = g_parsinginfo[0].GetParsingMatByName(label,"LaneMarking");
        cv::Mat channels = g_parsinginfo[0].GetParsingMatByNames(label,{ "Road" , "Car", "LaneMarking", "Truck"});
        cv::Mat lane=g_parsinginfo[1].GetParsingMat(label);

       // cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
        //cv_ptr->image=full;
        //ros_fullimage_pub.publish(cv_ptr->toImageMsg());
        if(is_pub_fullimage){
            sensor_msgs::ImagePtr msg_full = cv_bridge::CvImage(std_msgs::Header(), "bgr8", full).toImageMsg();
            fullimage_pub_custom[i].publish(msg_full);
        }

        //cv_ptr->image=one_channel;
       //ros_onechannel_pub.publish(cv_ptr->toImageMsg());
        if(is_pub_onechannelimage){
            sensor_msgs::ImagePtr msg_onechannel = cv_bridge::CvImage(std_msgs::Header(), "bgr8", one_channel).toImageMsg();
            onechannelimage_pub_custom[i].publish(msg_onechannel);
        }
        //cv_ptr->image=channels;
        //ros_channels_pub.publish(cv_ptr->toImageMsg());
        if(is_pub_channelsimage){
            sensor_msgs::ImagePtr msg_channels = cv_bridge::CvImage(std_msgs::Header(), "bgr8", channels).toImageMsg();
            channelsimage_pub_custom[i].publish(msg_channels);
        }
        if(is_pub_laneimage){
            sensor_msgs::ImagePtr msg_lane = cv_bridge::CvImage(std_msgs::Header(), "bgr8", lane).toImageMsg();
            laneimage_pub_custom[i].publish(msg_lane);
        }

        delete[] parsing_data;
        parsing_data = NULL;
      }
    }
  
    delete[] buffer;
    buffer = NULL;
  }
}

int main(int argc, char **argv) {
   ros::init(argc, argv, "matrix_process");
   ros::NodeHandle nh;
   ros::NodeHandle private_nh("~");
   image_transport::ImageTransport it(private_nh);
   ros::Subscriber sub = nh.subscribe("matrix_msg", 1, matrixCallback);


   camera_pub_custom=private_nh.advertise<matrix::Cameras>("/matrix/camer",1);

   private_nh.param("is_pub_line",is_pub_line,bool(true));
   lines_pub_custom[0]=private_nh.advertise<matrix::MatrixLanes>("/matrix/lines0",1);
   lines_pub_custom[1]=private_nh.advertise<matrix::MatrixLanes>("/matrix/lines1",1);
   lines_pub_custom[2]=private_nh.advertise<matrix::MatrixLanes>("/matrix/lines2",1);
   lines_pub_custom[3]=private_nh.advertise<matrix::MatrixLanes>("/matrix/lines3",1);

   private_nh.param("is_pub_object",is_pub_object,bool(true));
   object_pub_custom[0]=private_nh.advertise<matrix::MatrixObjects>("/matrix/object0",1);
   object_pub_custom[1]=private_nh.advertise<matrix::MatrixObjects>("/matrix/object1",1);
   object_pub_custom[2]=private_nh.advertise<matrix::MatrixObjects>("/matrix/object2",1);
   object_pub_custom[3]=private_nh.advertise<matrix::MatrixObjects>("/matrix/object3",1);

   private_nh.param("is_pub_light",is_pub_light,bool(true));
   trafficlight_pub_custom[0]=private_nh.advertise<matrix::TrafficLights>("/matrix/traffic_light0",1);
   trafficlight_pub_custom[1]=private_nh.advertise<matrix::TrafficLights>("/matrix/traffic_light1",1);
   trafficlight_pub_custom[2]=private_nh.advertise<matrix::TrafficLights>("/matrix/traffic_light2",1);
   trafficlight_pub_custom[3]=private_nh.advertise<matrix::TrafficLights>("/matrix/traffic_light3",1);

   private_nh.param("is_pub_fullimage",is_pub_fullimage,bool(true));
   fullimage_pub_custom[0]=it.advertise("/matrix/fullimage0",1);
   fullimage_pub_custom[1]=it.advertise("/matrix/fullimage1",1);
   fullimage_pub_custom[2]=it.advertise("/matrix/fullimage2",1);
   fullimage_pub_custom[3]=it.advertise("/matrix/fullimage3",1);

   private_nh.param("is_pub_onechannelimage",is_pub_onechannelimage,bool(true));
   onechannelimage_pub_custom[0]=it.advertise("/matrix/onechannelimage0",1);
   onechannelimage_pub_custom[1]=it.advertise("/matrix/onechannelimage1",1);
   onechannelimage_pub_custom[2]=it.advertise("/matrix/onechannelimage2",1);
   onechannelimage_pub_custom[3]=it.advertise("/matrix/onechannelimage3",1);

   private_nh.param("is_pub_channelsimage",is_pub_channelsimage,bool(true));
   channelsimage_pub_custom[0]=it.advertise("/matrix/channelsimage0",1);
   channelsimage_pub_custom[1]=it.advertise("/matrix/channelsimage1",1);
   channelsimage_pub_custom[2]=it.advertise("/matrix/channelsimage2",1);
   channelsimage_pub_custom[3]=it.advertise("/matrix/channelsimage3",1);

   private_nh.param("is_pub_laneimage",is_pub_laneimage,bool(true));
   laneimage_pub_custom[0]=it.advertise("/matrix/laneimage0",1);
   laneimage_pub_custom[1]=it.advertise("/matrix/laneimage1",1);
   laneimage_pub_custom[2]=it.advertise("/matrix/laneimage2",1);
   laneimage_pub_custom[3]=it.advertise("/matrix/laneimage3",1);

   private_nh.param("is_pub_image",is_pub_image,bool(true));
   image_pub_custom[0]=it.advertise("/matrix/image0",1);
   image_pub_custom[1]=it.advertise("/matrix/image1",1);
   image_pub_custom[2]=it.advertise("/matrix/image2",1);
   image_pub_custom[3]=it.advertise("/matrix/image3",1);

   private_nh.param("is_pub_trafficsign",is_pub_trafficsign,bool(true));
   trafficsign_pub_cutom[0]=private_nh.advertise<matrix::TrafficSigns>("/matrix/trafficsign0",1);
   trafficsign_pub_cutom[1]=private_nh.advertise<matrix::TrafficSigns>("/matrix/trafficsign1",1);
   trafficsign_pub_cutom[2]=private_nh.advertise<matrix::TrafficSigns>("/matrix/trafficsign2",1);
   trafficsign_pub_cutom[3]=private_nh.advertise<matrix::TrafficSigns>("/matrix/trafficsign3",1);
   ros::spin();
   return 0;
}
