// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: sensor.proto

#ifndef PROTOBUF_sensor_2eproto__INCLUDED
#define PROTOBUF_sensor_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 2006000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 2006001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)

namespace SensorProto {

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_sensor_2eproto();
void protobuf_AssignDesc_sensor_2eproto();
void protobuf_ShutdownFile_sensor_2eproto();

class IMUFrame;
class GPSFrame;
class GPSFrameRaw;

// ===================================================================

class IMUFrame : public ::google::protobuf::Message {
 public:
  IMUFrame();
  virtual ~IMUFrame();

  IMUFrame(const IMUFrame& from);

  inline IMUFrame& operator=(const IMUFrame& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const IMUFrame& default_instance();

  void Swap(IMUFrame* other);

  // implements Message ----------------------------------------------

  IMUFrame* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const IMUFrame& from);
  void MergeFrom(const IMUFrame& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:
  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional float acc_x = 1;
  inline bool has_acc_x() const;
  inline void clear_acc_x();
  static const int kAccXFieldNumber = 1;
  inline float acc_x() const;
  inline void set_acc_x(float value);

  // optional float acc_y = 2;
  inline bool has_acc_y() const;
  inline void clear_acc_y();
  static const int kAccYFieldNumber = 2;
  inline float acc_y() const;
  inline void set_acc_y(float value);

  // optional float acc_z = 3;
  inline bool has_acc_z() const;
  inline void clear_acc_z();
  static const int kAccZFieldNumber = 3;
  inline float acc_z() const;
  inline void set_acc_z(float value);

  // optional float gyro_x = 4;
  inline bool has_gyro_x() const;
  inline void clear_gyro_x();
  static const int kGyroXFieldNumber = 4;
  inline float gyro_x() const;
  inline void set_gyro_x(float value);

  // optional float gyro_y = 5;
  inline bool has_gyro_y() const;
  inline void clear_gyro_y();
  static const int kGyroYFieldNumber = 5;
  inline float gyro_y() const;
  inline void set_gyro_y(float value);

  // optional float gyro_z = 6;
  inline bool has_gyro_z() const;
  inline void clear_gyro_z();
  static const int kGyroZFieldNumber = 6;
  inline float gyro_z() const;
  inline void set_gyro_z(float value);

  // optional float temperature = 7;
  inline bool has_temperature() const;
  inline void clear_temperature();
  static const int kTemperatureFieldNumber = 7;
  inline float temperature() const;
  inline void set_temperature(float value);

  // required int64 time_stamp = 8;
  inline bool has_time_stamp() const;
  inline void clear_time_stamp();
  static const int kTimeStampFieldNumber = 8;
  inline ::google::protobuf::int64 time_stamp() const;
  inline void set_time_stamp(::google::protobuf::int64 value);

  // @@protoc_insertion_point(class_scope:SensorProto.IMUFrame)
 private:
  inline void set_has_acc_x();
  inline void clear_has_acc_x();
  inline void set_has_acc_y();
  inline void clear_has_acc_y();
  inline void set_has_acc_z();
  inline void clear_has_acc_z();
  inline void set_has_gyro_x();
  inline void clear_has_gyro_x();
  inline void set_has_gyro_y();
  inline void clear_has_gyro_y();
  inline void set_has_gyro_z();
  inline void clear_has_gyro_z();
  inline void set_has_temperature();
  inline void clear_has_temperature();
  inline void set_has_time_stamp();
  inline void clear_has_time_stamp();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  float acc_x_;
  float acc_y_;
  float acc_z_;
  float gyro_x_;
  float gyro_y_;
  float gyro_z_;
  ::google::protobuf::int64 time_stamp_;
  float temperature_;
  friend void  protobuf_AddDesc_sensor_2eproto();
  friend void protobuf_AssignDesc_sensor_2eproto();
  friend void protobuf_ShutdownFile_sensor_2eproto();

  void InitAsDefaultInstance();
  static IMUFrame* default_instance_;
};
// -------------------------------------------------------------------

class GPSFrame : public ::google::protobuf::Message {
 public:
  GPSFrame();
  virtual ~GPSFrame();

  GPSFrame(const GPSFrame& from);

  inline GPSFrame& operator=(const GPSFrame& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const GPSFrame& default_instance();

  void Swap(GPSFrame* other);

  // implements Message ----------------------------------------------

  GPSFrame* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const GPSFrame& from);
  void MergeFrom(const GPSFrame& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:
  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // required float longitude = 1;
  inline bool has_longitude() const;
  inline void clear_longitude();
  static const int kLongitudeFieldNumber = 1;
  inline float longitude() const;
  inline void set_longitude(float value);

  // required float latitude = 2;
  inline bool has_latitude() const;
  inline void clear_latitude();
  static const int kLatitudeFieldNumber = 2;
  inline float latitude() const;
  inline void set_latitude(float value);

  // required int64 time_stamp = 3;
  inline bool has_time_stamp() const;
  inline void clear_time_stamp();
  static const int kTimeStampFieldNumber = 3;
  inline ::google::protobuf::int64 time_stamp() const;
  inline void set_time_stamp(::google::protobuf::int64 value);

  // optional float longitude_cent = 4;
  inline bool has_longitude_cent() const;
  inline void clear_longitude_cent();
  static const int kLongitudeCentFieldNumber = 4;
  inline float longitude_cent() const;
  inline void set_longitude_cent(float value);

  // optional string longitude_dir = 5;
  inline bool has_longitude_dir() const;
  inline void clear_longitude_dir();
  static const int kLongitudeDirFieldNumber = 5;
  inline const ::std::string& longitude_dir() const;
  inline void set_longitude_dir(const ::std::string& value);
  inline void set_longitude_dir(const char* value);
  inline void set_longitude_dir(const char* value, size_t size);
  inline ::std::string* mutable_longitude_dir();
  inline ::std::string* release_longitude_dir();
  inline void set_allocated_longitude_dir(::std::string* longitude_dir);

  // optional float latitude_cent = 6;
  inline bool has_latitude_cent() const;
  inline void clear_latitude_cent();
  static const int kLatitudeCentFieldNumber = 6;
  inline float latitude_cent() const;
  inline void set_latitude_cent(float value);

  // optional string latitude_dir = 7;
  inline bool has_latitude_dir() const;
  inline void clear_latitude_dir();
  static const int kLatitudeDirFieldNumber = 7;
  inline const ::std::string& latitude_dir() const;
  inline void set_latitude_dir(const ::std::string& value);
  inline void set_latitude_dir(const char* value);
  inline void set_latitude_dir(const char* value, size_t size);
  inline ::std::string* mutable_latitude_dir();
  inline ::std::string* release_latitude_dir();
  inline void set_allocated_latitude_dir(::std::string* latitude_dir);

  // optional float ground_speed = 8;
  inline bool has_ground_speed() const;
  inline void clear_ground_speed();
  static const int kGroundSpeedFieldNumber = 8;
  inline float ground_speed() const;
  inline void set_ground_speed(float value);

  // optional float ground_course = 9;
  inline bool has_ground_course() const;
  inline void clear_ground_course();
  static const int kGroundCourseFieldNumber = 9;
  inline float ground_course() const;
  inline void set_ground_course(float value);

  // optional int64 gps_time = 10;
  inline bool has_gps_time() const;
  inline void clear_gps_time();
  static const int kGpsTimeFieldNumber = 10;
  inline ::google::protobuf::int64 gps_time() const;
  inline void set_gps_time(::google::protobuf::int64 value);

  // optional float altitude = 11;
  inline bool has_altitude() const;
  inline void clear_altitude();
  static const int kAltitudeFieldNumber = 11;
  inline float altitude() const;
  inline void set_altitude(float value);

  // optional float accuracy = 12;
  inline bool has_accuracy() const;
  inline void clear_accuracy();
  static const int kAccuracyFieldNumber = 12;
  inline float accuracy() const;
  inline void set_accuracy(float value);

  // @@protoc_insertion_point(class_scope:SensorProto.GPSFrame)
 private:
  inline void set_has_longitude();
  inline void clear_has_longitude();
  inline void set_has_latitude();
  inline void clear_has_latitude();
  inline void set_has_time_stamp();
  inline void clear_has_time_stamp();
  inline void set_has_longitude_cent();
  inline void clear_has_longitude_cent();
  inline void set_has_longitude_dir();
  inline void clear_has_longitude_dir();
  inline void set_has_latitude_cent();
  inline void clear_has_latitude_cent();
  inline void set_has_latitude_dir();
  inline void clear_has_latitude_dir();
  inline void set_has_ground_speed();
  inline void clear_has_ground_speed();
  inline void set_has_ground_course();
  inline void clear_has_ground_course();
  inline void set_has_gps_time();
  inline void clear_has_gps_time();
  inline void set_has_altitude();
  inline void clear_has_altitude();
  inline void set_has_accuracy();
  inline void clear_has_accuracy();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  float longitude_;
  float latitude_;
  ::google::protobuf::int64 time_stamp_;
  ::std::string* longitude_dir_;
  float longitude_cent_;
  float latitude_cent_;
  ::std::string* latitude_dir_;
  float ground_speed_;
  float ground_course_;
  ::google::protobuf::int64 gps_time_;
  float altitude_;
  float accuracy_;
  friend void  protobuf_AddDesc_sensor_2eproto();
  friend void protobuf_AssignDesc_sensor_2eproto();
  friend void protobuf_ShutdownFile_sensor_2eproto();

  void InitAsDefaultInstance();
  static GPSFrame* default_instance_;
};
// -------------------------------------------------------------------

class GPSFrameRaw : public ::google::protobuf::Message {
 public:
  GPSFrameRaw();
  virtual ~GPSFrameRaw();

  GPSFrameRaw(const GPSFrameRaw& from);

  inline GPSFrameRaw& operator=(const GPSFrameRaw& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const GPSFrameRaw& default_instance();

  void Swap(GPSFrameRaw* other);

  // implements Message ----------------------------------------------

  GPSFrameRaw* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const GPSFrameRaw& from);
  void MergeFrom(const GPSFrameRaw& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:
  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional string info = 1;
  inline bool has_info() const;
  inline void clear_info();
  static const int kInfoFieldNumber = 1;
  inline const ::std::string& info() const;
  inline void set_info(const ::std::string& value);
  inline void set_info(const char* value);
  inline void set_info(const char* value, size_t size);
  inline ::std::string* mutable_info();
  inline ::std::string* release_info();
  inline void set_allocated_info(::std::string* info);

  // required int64 time_stamp = 2;
  inline bool has_time_stamp() const;
  inline void clear_time_stamp();
  static const int kTimeStampFieldNumber = 2;
  inline ::google::protobuf::int64 time_stamp() const;
  inline void set_time_stamp(::google::protobuf::int64 value);

  // optional .SensorProto.GPSFrame parsed = 3;
  inline bool has_parsed() const;
  inline void clear_parsed();
  static const int kParsedFieldNumber = 3;
  inline const ::SensorProto::GPSFrame& parsed() const;
  inline ::SensorProto::GPSFrame* mutable_parsed();
  inline ::SensorProto::GPSFrame* release_parsed();
  inline void set_allocated_parsed(::SensorProto::GPSFrame* parsed);

  // @@protoc_insertion_point(class_scope:SensorProto.GPSFrameRaw)
 private:
  inline void set_has_info();
  inline void clear_has_info();
  inline void set_has_time_stamp();
  inline void clear_has_time_stamp();
  inline void set_has_parsed();
  inline void clear_has_parsed();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::std::string* info_;
  ::google::protobuf::int64 time_stamp_;
  ::SensorProto::GPSFrame* parsed_;
  friend void  protobuf_AddDesc_sensor_2eproto();
  friend void protobuf_AssignDesc_sensor_2eproto();
  friend void protobuf_ShutdownFile_sensor_2eproto();

  void InitAsDefaultInstance();
  static GPSFrameRaw* default_instance_;
};
// ===================================================================


// ===================================================================

// IMUFrame

// optional float acc_x = 1;
inline bool IMUFrame::has_acc_x() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void IMUFrame::set_has_acc_x() {
  _has_bits_[0] |= 0x00000001u;
}
inline void IMUFrame::clear_has_acc_x() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void IMUFrame::clear_acc_x() {
  acc_x_ = 0;
  clear_has_acc_x();
}
inline float IMUFrame::acc_x() const {
  // @@protoc_insertion_point(field_get:SensorProto.IMUFrame.acc_x)
  return acc_x_;
}
inline void IMUFrame::set_acc_x(float value) {
  set_has_acc_x();
  acc_x_ = value;
  // @@protoc_insertion_point(field_set:SensorProto.IMUFrame.acc_x)
}

// optional float acc_y = 2;
inline bool IMUFrame::has_acc_y() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void IMUFrame::set_has_acc_y() {
  _has_bits_[0] |= 0x00000002u;
}
inline void IMUFrame::clear_has_acc_y() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void IMUFrame::clear_acc_y() {
  acc_y_ = 0;
  clear_has_acc_y();
}
inline float IMUFrame::acc_y() const {
  // @@protoc_insertion_point(field_get:SensorProto.IMUFrame.acc_y)
  return acc_y_;
}
inline void IMUFrame::set_acc_y(float value) {
  set_has_acc_y();
  acc_y_ = value;
  // @@protoc_insertion_point(field_set:SensorProto.IMUFrame.acc_y)
}

// optional float acc_z = 3;
inline bool IMUFrame::has_acc_z() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void IMUFrame::set_has_acc_z() {
  _has_bits_[0] |= 0x00000004u;
}
inline void IMUFrame::clear_has_acc_z() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void IMUFrame::clear_acc_z() {
  acc_z_ = 0;
  clear_has_acc_z();
}
inline float IMUFrame::acc_z() const {
  // @@protoc_insertion_point(field_get:SensorProto.IMUFrame.acc_z)
  return acc_z_;
}
inline void IMUFrame::set_acc_z(float value) {
  set_has_acc_z();
  acc_z_ = value;
  // @@protoc_insertion_point(field_set:SensorProto.IMUFrame.acc_z)
}

// optional float gyro_x = 4;
inline bool IMUFrame::has_gyro_x() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void IMUFrame::set_has_gyro_x() {
  _has_bits_[0] |= 0x00000008u;
}
inline void IMUFrame::clear_has_gyro_x() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void IMUFrame::clear_gyro_x() {
  gyro_x_ = 0;
  clear_has_gyro_x();
}
inline float IMUFrame::gyro_x() const {
  // @@protoc_insertion_point(field_get:SensorProto.IMUFrame.gyro_x)
  return gyro_x_;
}
inline void IMUFrame::set_gyro_x(float value) {
  set_has_gyro_x();
  gyro_x_ = value;
  // @@protoc_insertion_point(field_set:SensorProto.IMUFrame.gyro_x)
}

// optional float gyro_y = 5;
inline bool IMUFrame::has_gyro_y() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void IMUFrame::set_has_gyro_y() {
  _has_bits_[0] |= 0x00000010u;
}
inline void IMUFrame::clear_has_gyro_y() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void IMUFrame::clear_gyro_y() {
  gyro_y_ = 0;
  clear_has_gyro_y();
}
inline float IMUFrame::gyro_y() const {
  // @@protoc_insertion_point(field_get:SensorProto.IMUFrame.gyro_y)
  return gyro_y_;
}
inline void IMUFrame::set_gyro_y(float value) {
  set_has_gyro_y();
  gyro_y_ = value;
  // @@protoc_insertion_point(field_set:SensorProto.IMUFrame.gyro_y)
}

// optional float gyro_z = 6;
inline bool IMUFrame::has_gyro_z() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void IMUFrame::set_has_gyro_z() {
  _has_bits_[0] |= 0x00000020u;
}
inline void IMUFrame::clear_has_gyro_z() {
  _has_bits_[0] &= ~0x00000020u;
}
inline void IMUFrame::clear_gyro_z() {
  gyro_z_ = 0;
  clear_has_gyro_z();
}
inline float IMUFrame::gyro_z() const {
  // @@protoc_insertion_point(field_get:SensorProto.IMUFrame.gyro_z)
  return gyro_z_;
}
inline void IMUFrame::set_gyro_z(float value) {
  set_has_gyro_z();
  gyro_z_ = value;
  // @@protoc_insertion_point(field_set:SensorProto.IMUFrame.gyro_z)
}

// optional float temperature = 7;
inline bool IMUFrame::has_temperature() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
inline void IMUFrame::set_has_temperature() {
  _has_bits_[0] |= 0x00000040u;
}
inline void IMUFrame::clear_has_temperature() {
  _has_bits_[0] &= ~0x00000040u;
}
inline void IMUFrame::clear_temperature() {
  temperature_ = 0;
  clear_has_temperature();
}
inline float IMUFrame::temperature() const {
  // @@protoc_insertion_point(field_get:SensorProto.IMUFrame.temperature)
  return temperature_;
}
inline void IMUFrame::set_temperature(float value) {
  set_has_temperature();
  temperature_ = value;
  // @@protoc_insertion_point(field_set:SensorProto.IMUFrame.temperature)
}

// required int64 time_stamp = 8;
inline bool IMUFrame::has_time_stamp() const {
  return (_has_bits_[0] & 0x00000080u) != 0;
}
inline void IMUFrame::set_has_time_stamp() {
  _has_bits_[0] |= 0x00000080u;
}
inline void IMUFrame::clear_has_time_stamp() {
  _has_bits_[0] &= ~0x00000080u;
}
inline void IMUFrame::clear_time_stamp() {
  time_stamp_ = GOOGLE_LONGLONG(0);
  clear_has_time_stamp();
}
inline ::google::protobuf::int64 IMUFrame::time_stamp() const {
  // @@protoc_insertion_point(field_get:SensorProto.IMUFrame.time_stamp)
  return time_stamp_;
}
inline void IMUFrame::set_time_stamp(::google::protobuf::int64 value) {
  set_has_time_stamp();
  time_stamp_ = value;
  // @@protoc_insertion_point(field_set:SensorProto.IMUFrame.time_stamp)
}

// -------------------------------------------------------------------

// GPSFrame

// required float longitude = 1;
inline bool GPSFrame::has_longitude() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void GPSFrame::set_has_longitude() {
  _has_bits_[0] |= 0x00000001u;
}
inline void GPSFrame::clear_has_longitude() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void GPSFrame::clear_longitude() {
  longitude_ = 0;
  clear_has_longitude();
}
inline float GPSFrame::longitude() const {
  // @@protoc_insertion_point(field_get:SensorProto.GPSFrame.longitude)
  return longitude_;
}
inline void GPSFrame::set_longitude(float value) {
  set_has_longitude();
  longitude_ = value;
  // @@protoc_insertion_point(field_set:SensorProto.GPSFrame.longitude)
}

// required float latitude = 2;
inline bool GPSFrame::has_latitude() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void GPSFrame::set_has_latitude() {
  _has_bits_[0] |= 0x00000002u;
}
inline void GPSFrame::clear_has_latitude() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void GPSFrame::clear_latitude() {
  latitude_ = 0;
  clear_has_latitude();
}
inline float GPSFrame::latitude() const {
  // @@protoc_insertion_point(field_get:SensorProto.GPSFrame.latitude)
  return latitude_;
}
inline void GPSFrame::set_latitude(float value) {
  set_has_latitude();
  latitude_ = value;
  // @@protoc_insertion_point(field_set:SensorProto.GPSFrame.latitude)
}

// required int64 time_stamp = 3;
inline bool GPSFrame::has_time_stamp() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void GPSFrame::set_has_time_stamp() {
  _has_bits_[0] |= 0x00000004u;
}
inline void GPSFrame::clear_has_time_stamp() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void GPSFrame::clear_time_stamp() {
  time_stamp_ = GOOGLE_LONGLONG(0);
  clear_has_time_stamp();
}
inline ::google::protobuf::int64 GPSFrame::time_stamp() const {
  // @@protoc_insertion_point(field_get:SensorProto.GPSFrame.time_stamp)
  return time_stamp_;
}
inline void GPSFrame::set_time_stamp(::google::protobuf::int64 value) {
  set_has_time_stamp();
  time_stamp_ = value;
  // @@protoc_insertion_point(field_set:SensorProto.GPSFrame.time_stamp)
}

// optional float longitude_cent = 4;
inline bool GPSFrame::has_longitude_cent() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void GPSFrame::set_has_longitude_cent() {
  _has_bits_[0] |= 0x00000008u;
}
inline void GPSFrame::clear_has_longitude_cent() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void GPSFrame::clear_longitude_cent() {
  longitude_cent_ = 0;
  clear_has_longitude_cent();
}
inline float GPSFrame::longitude_cent() const {
  // @@protoc_insertion_point(field_get:SensorProto.GPSFrame.longitude_cent)
  return longitude_cent_;
}
inline void GPSFrame::set_longitude_cent(float value) {
  set_has_longitude_cent();
  longitude_cent_ = value;
  // @@protoc_insertion_point(field_set:SensorProto.GPSFrame.longitude_cent)
}

// optional string longitude_dir = 5;
inline bool GPSFrame::has_longitude_dir() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void GPSFrame::set_has_longitude_dir() {
  _has_bits_[0] |= 0x00000010u;
}
inline void GPSFrame::clear_has_longitude_dir() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void GPSFrame::clear_longitude_dir() {
  if (longitude_dir_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    longitude_dir_->clear();
  }
  clear_has_longitude_dir();
}
inline const ::std::string& GPSFrame::longitude_dir() const {
  // @@protoc_insertion_point(field_get:SensorProto.GPSFrame.longitude_dir)
  return *longitude_dir_;
}
inline void GPSFrame::set_longitude_dir(const ::std::string& value) {
  set_has_longitude_dir();
  if (longitude_dir_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    longitude_dir_ = new ::std::string;
  }
  longitude_dir_->assign(value);
  // @@protoc_insertion_point(field_set:SensorProto.GPSFrame.longitude_dir)
}
inline void GPSFrame::set_longitude_dir(const char* value) {
  set_has_longitude_dir();
  if (longitude_dir_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    longitude_dir_ = new ::std::string;
  }
  longitude_dir_->assign(value);
  // @@protoc_insertion_point(field_set_char:SensorProto.GPSFrame.longitude_dir)
}
inline void GPSFrame::set_longitude_dir(const char* value, size_t size) {
  set_has_longitude_dir();
  if (longitude_dir_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    longitude_dir_ = new ::std::string;
  }
  longitude_dir_->assign(reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_set_pointer:SensorProto.GPSFrame.longitude_dir)
}
inline ::std::string* GPSFrame::mutable_longitude_dir() {
  set_has_longitude_dir();
  if (longitude_dir_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    longitude_dir_ = new ::std::string;
  }
  // @@protoc_insertion_point(field_mutable:SensorProto.GPSFrame.longitude_dir)
  return longitude_dir_;
}
inline ::std::string* GPSFrame::release_longitude_dir() {
  clear_has_longitude_dir();
  if (longitude_dir_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    return NULL;
  } else {
    ::std::string* temp = longitude_dir_;
    longitude_dir_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
    return temp;
  }
}
inline void GPSFrame::set_allocated_longitude_dir(::std::string* longitude_dir) {
  if (longitude_dir_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    delete longitude_dir_;
  }
  if (longitude_dir) {
    set_has_longitude_dir();
    longitude_dir_ = longitude_dir;
  } else {
    clear_has_longitude_dir();
    longitude_dir_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  }
  // @@protoc_insertion_point(field_set_allocated:SensorProto.GPSFrame.longitude_dir)
}

// optional float latitude_cent = 6;
inline bool GPSFrame::has_latitude_cent() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void GPSFrame::set_has_latitude_cent() {
  _has_bits_[0] |= 0x00000020u;
}
inline void GPSFrame::clear_has_latitude_cent() {
  _has_bits_[0] &= ~0x00000020u;
}
inline void GPSFrame::clear_latitude_cent() {
  latitude_cent_ = 0;
  clear_has_latitude_cent();
}
inline float GPSFrame::latitude_cent() const {
  // @@protoc_insertion_point(field_get:SensorProto.GPSFrame.latitude_cent)
  return latitude_cent_;
}
inline void GPSFrame::set_latitude_cent(float value) {
  set_has_latitude_cent();
  latitude_cent_ = value;
  // @@protoc_insertion_point(field_set:SensorProto.GPSFrame.latitude_cent)
}

// optional string latitude_dir = 7;
inline bool GPSFrame::has_latitude_dir() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
inline void GPSFrame::set_has_latitude_dir() {
  _has_bits_[0] |= 0x00000040u;
}
inline void GPSFrame::clear_has_latitude_dir() {
  _has_bits_[0] &= ~0x00000040u;
}
inline void GPSFrame::clear_latitude_dir() {
  if (latitude_dir_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    latitude_dir_->clear();
  }
  clear_has_latitude_dir();
}
inline const ::std::string& GPSFrame::latitude_dir() const {
  // @@protoc_insertion_point(field_get:SensorProto.GPSFrame.latitude_dir)
  return *latitude_dir_;
}
inline void GPSFrame::set_latitude_dir(const ::std::string& value) {
  set_has_latitude_dir();
  if (latitude_dir_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    latitude_dir_ = new ::std::string;
  }
  latitude_dir_->assign(value);
  // @@protoc_insertion_point(field_set:SensorProto.GPSFrame.latitude_dir)
}
inline void GPSFrame::set_latitude_dir(const char* value) {
  set_has_latitude_dir();
  if (latitude_dir_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    latitude_dir_ = new ::std::string;
  }
  latitude_dir_->assign(value);
  // @@protoc_insertion_point(field_set_char:SensorProto.GPSFrame.latitude_dir)
}
inline void GPSFrame::set_latitude_dir(const char* value, size_t size) {
  set_has_latitude_dir();
  if (latitude_dir_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    latitude_dir_ = new ::std::string;
  }
  latitude_dir_->assign(reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_set_pointer:SensorProto.GPSFrame.latitude_dir)
}
inline ::std::string* GPSFrame::mutable_latitude_dir() {
  set_has_latitude_dir();
  if (latitude_dir_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    latitude_dir_ = new ::std::string;
  }
  // @@protoc_insertion_point(field_mutable:SensorProto.GPSFrame.latitude_dir)
  return latitude_dir_;
}
inline ::std::string* GPSFrame::release_latitude_dir() {
  clear_has_latitude_dir();
  if (latitude_dir_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    return NULL;
  } else {
    ::std::string* temp = latitude_dir_;
    latitude_dir_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
    return temp;
  }
}
inline void GPSFrame::set_allocated_latitude_dir(::std::string* latitude_dir) {
  if (latitude_dir_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    delete latitude_dir_;
  }
  if (latitude_dir) {
    set_has_latitude_dir();
    latitude_dir_ = latitude_dir;
  } else {
    clear_has_latitude_dir();
    latitude_dir_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  }
  // @@protoc_insertion_point(field_set_allocated:SensorProto.GPSFrame.latitude_dir)
}

// optional float ground_speed = 8;
inline bool GPSFrame::has_ground_speed() const {
  return (_has_bits_[0] & 0x00000080u) != 0;
}
inline void GPSFrame::set_has_ground_speed() {
  _has_bits_[0] |= 0x00000080u;
}
inline void GPSFrame::clear_has_ground_speed() {
  _has_bits_[0] &= ~0x00000080u;
}
inline void GPSFrame::clear_ground_speed() {
  ground_speed_ = 0;
  clear_has_ground_speed();
}
inline float GPSFrame::ground_speed() const {
  // @@protoc_insertion_point(field_get:SensorProto.GPSFrame.ground_speed)
  return ground_speed_;
}
inline void GPSFrame::set_ground_speed(float value) {
  set_has_ground_speed();
  ground_speed_ = value;
  // @@protoc_insertion_point(field_set:SensorProto.GPSFrame.ground_speed)
}

// optional float ground_course = 9;
inline bool GPSFrame::has_ground_course() const {
  return (_has_bits_[0] & 0x00000100u) != 0;
}
inline void GPSFrame::set_has_ground_course() {
  _has_bits_[0] |= 0x00000100u;
}
inline void GPSFrame::clear_has_ground_course() {
  _has_bits_[0] &= ~0x00000100u;
}
inline void GPSFrame::clear_ground_course() {
  ground_course_ = 0;
  clear_has_ground_course();
}
inline float GPSFrame::ground_course() const {
  // @@protoc_insertion_point(field_get:SensorProto.GPSFrame.ground_course)
  return ground_course_;
}
inline void GPSFrame::set_ground_course(float value) {
  set_has_ground_course();
  ground_course_ = value;
  // @@protoc_insertion_point(field_set:SensorProto.GPSFrame.ground_course)
}

// optional int64 gps_time = 10;
inline bool GPSFrame::has_gps_time() const {
  return (_has_bits_[0] & 0x00000200u) != 0;
}
inline void GPSFrame::set_has_gps_time() {
  _has_bits_[0] |= 0x00000200u;
}
inline void GPSFrame::clear_has_gps_time() {
  _has_bits_[0] &= ~0x00000200u;
}
inline void GPSFrame::clear_gps_time() {
  gps_time_ = GOOGLE_LONGLONG(0);
  clear_has_gps_time();
}
inline ::google::protobuf::int64 GPSFrame::gps_time() const {
  // @@protoc_insertion_point(field_get:SensorProto.GPSFrame.gps_time)
  return gps_time_;
}
inline void GPSFrame::set_gps_time(::google::protobuf::int64 value) {
  set_has_gps_time();
  gps_time_ = value;
  // @@protoc_insertion_point(field_set:SensorProto.GPSFrame.gps_time)
}

// optional float altitude = 11;
inline bool GPSFrame::has_altitude() const {
  return (_has_bits_[0] & 0x00000400u) != 0;
}
inline void GPSFrame::set_has_altitude() {
  _has_bits_[0] |= 0x00000400u;
}
inline void GPSFrame::clear_has_altitude() {
  _has_bits_[0] &= ~0x00000400u;
}
inline void GPSFrame::clear_altitude() {
  altitude_ = 0;
  clear_has_altitude();
}
inline float GPSFrame::altitude() const {
  // @@protoc_insertion_point(field_get:SensorProto.GPSFrame.altitude)
  return altitude_;
}
inline void GPSFrame::set_altitude(float value) {
  set_has_altitude();
  altitude_ = value;
  // @@protoc_insertion_point(field_set:SensorProto.GPSFrame.altitude)
}

// optional float accuracy = 12;
inline bool GPSFrame::has_accuracy() const {
  return (_has_bits_[0] & 0x00000800u) != 0;
}
inline void GPSFrame::set_has_accuracy() {
  _has_bits_[0] |= 0x00000800u;
}
inline void GPSFrame::clear_has_accuracy() {
  _has_bits_[0] &= ~0x00000800u;
}
inline void GPSFrame::clear_accuracy() {
  accuracy_ = 0;
  clear_has_accuracy();
}
inline float GPSFrame::accuracy() const {
  // @@protoc_insertion_point(field_get:SensorProto.GPSFrame.accuracy)
  return accuracy_;
}
inline void GPSFrame::set_accuracy(float value) {
  set_has_accuracy();
  accuracy_ = value;
  // @@protoc_insertion_point(field_set:SensorProto.GPSFrame.accuracy)
}

// -------------------------------------------------------------------

// GPSFrameRaw

// optional string info = 1;
inline bool GPSFrameRaw::has_info() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void GPSFrameRaw::set_has_info() {
  _has_bits_[0] |= 0x00000001u;
}
inline void GPSFrameRaw::clear_has_info() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void GPSFrameRaw::clear_info() {
  if (info_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    info_->clear();
  }
  clear_has_info();
}
inline const ::std::string& GPSFrameRaw::info() const {
  // @@protoc_insertion_point(field_get:SensorProto.GPSFrameRaw.info)
  return *info_;
}
inline void GPSFrameRaw::set_info(const ::std::string& value) {
  set_has_info();
  if (info_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    info_ = new ::std::string;
  }
  info_->assign(value);
  // @@protoc_insertion_point(field_set:SensorProto.GPSFrameRaw.info)
}
inline void GPSFrameRaw::set_info(const char* value) {
  set_has_info();
  if (info_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    info_ = new ::std::string;
  }
  info_->assign(value);
  // @@protoc_insertion_point(field_set_char:SensorProto.GPSFrameRaw.info)
}
inline void GPSFrameRaw::set_info(const char* value, size_t size) {
  set_has_info();
  if (info_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    info_ = new ::std::string;
  }
  info_->assign(reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_set_pointer:SensorProto.GPSFrameRaw.info)
}
inline ::std::string* GPSFrameRaw::mutable_info() {
  set_has_info();
  if (info_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    info_ = new ::std::string;
  }
  // @@protoc_insertion_point(field_mutable:SensorProto.GPSFrameRaw.info)
  return info_;
}
inline ::std::string* GPSFrameRaw::release_info() {
  clear_has_info();
  if (info_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    return NULL;
  } else {
    ::std::string* temp = info_;
    info_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
    return temp;
  }
}
inline void GPSFrameRaw::set_allocated_info(::std::string* info) {
  if (info_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    delete info_;
  }
  if (info) {
    set_has_info();
    info_ = info;
  } else {
    clear_has_info();
    info_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  }
  // @@protoc_insertion_point(field_set_allocated:SensorProto.GPSFrameRaw.info)
}

// required int64 time_stamp = 2;
inline bool GPSFrameRaw::has_time_stamp() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void GPSFrameRaw::set_has_time_stamp() {
  _has_bits_[0] |= 0x00000002u;
}
inline void GPSFrameRaw::clear_has_time_stamp() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void GPSFrameRaw::clear_time_stamp() {
  time_stamp_ = GOOGLE_LONGLONG(0);
  clear_has_time_stamp();
}
inline ::google::protobuf::int64 GPSFrameRaw::time_stamp() const {
  // @@protoc_insertion_point(field_get:SensorProto.GPSFrameRaw.time_stamp)
  return time_stamp_;
}
inline void GPSFrameRaw::set_time_stamp(::google::protobuf::int64 value) {
  set_has_time_stamp();
  time_stamp_ = value;
  // @@protoc_insertion_point(field_set:SensorProto.GPSFrameRaw.time_stamp)
}

// optional .SensorProto.GPSFrame parsed = 3;
inline bool GPSFrameRaw::has_parsed() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void GPSFrameRaw::set_has_parsed() {
  _has_bits_[0] |= 0x00000004u;
}
inline void GPSFrameRaw::clear_has_parsed() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void GPSFrameRaw::clear_parsed() {
  if (parsed_ != NULL) parsed_->::SensorProto::GPSFrame::Clear();
  clear_has_parsed();
}
inline const ::SensorProto::GPSFrame& GPSFrameRaw::parsed() const {
  // @@protoc_insertion_point(field_get:SensorProto.GPSFrameRaw.parsed)
  return parsed_ != NULL ? *parsed_ : *default_instance_->parsed_;
}
inline ::SensorProto::GPSFrame* GPSFrameRaw::mutable_parsed() {
  set_has_parsed();
  if (parsed_ == NULL) parsed_ = new ::SensorProto::GPSFrame;
  // @@protoc_insertion_point(field_mutable:SensorProto.GPSFrameRaw.parsed)
  return parsed_;
}
inline ::SensorProto::GPSFrame* GPSFrameRaw::release_parsed() {
  clear_has_parsed();
  ::SensorProto::GPSFrame* temp = parsed_;
  parsed_ = NULL;
  return temp;
}
inline void GPSFrameRaw::set_allocated_parsed(::SensorProto::GPSFrame* parsed) {
  delete parsed_;
  parsed_ = parsed;
  if (parsed) {
    set_has_parsed();
  } else {
    clear_has_parsed();
  }
  // @@protoc_insertion_point(field_set_allocated:SensorProto.GPSFrameRaw.parsed)
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace SensorProto

#ifndef SWIG
namespace google {
namespace protobuf {


}  // namespace google
}  // namespace protobuf
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_sensor_2eproto__INCLUDED
