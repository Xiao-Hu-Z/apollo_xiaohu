// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: pointcloud_preprocessor_config.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "pointcloud_preprocessor_config.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)

namespace apollo {
namespace perception {
namespace lidar {

namespace {

const ::google::protobuf::Descriptor* PointCloudPreprocessorConfig_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  PointCloudPreprocessorConfig_reflection_ = NULL;

}  // namespace


void protobuf_AssignDesc_pointcloud_5fpreprocessor_5fconfig_2eproto() GOOGLE_ATTRIBUTE_COLD;
void protobuf_AssignDesc_pointcloud_5fpreprocessor_5fconfig_2eproto() {
  protobuf_AddDesc_pointcloud_5fpreprocessor_5fconfig_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "pointcloud_preprocessor_config.proto");
  GOOGLE_CHECK(file != NULL);
  PointCloudPreprocessorConfig_descriptor_ = file->message_type(0);
  static const int PointCloudPreprocessorConfig_offsets_[8] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(PointCloudPreprocessorConfig, filter_naninf_points_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(PointCloudPreprocessorConfig, filter_nearby_box_points_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(PointCloudPreprocessorConfig, box_forward_x_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(PointCloudPreprocessorConfig, box_backward_x_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(PointCloudPreprocessorConfig, box_forward_y_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(PointCloudPreprocessorConfig, box_backward_y_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(PointCloudPreprocessorConfig, filter_high_z_points_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(PointCloudPreprocessorConfig, z_threshold_),
  };
  PointCloudPreprocessorConfig_reflection_ =
    ::google::protobuf::internal::GeneratedMessageReflection::NewGeneratedMessageReflection(
      PointCloudPreprocessorConfig_descriptor_,
      PointCloudPreprocessorConfig::default_instance_,
      PointCloudPreprocessorConfig_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(PointCloudPreprocessorConfig, _has_bits_[0]),
      -1,
      -1,
      sizeof(PointCloudPreprocessorConfig),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(PointCloudPreprocessorConfig, _internal_metadata_),
      -1);
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
inline void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_pointcloud_5fpreprocessor_5fconfig_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
      PointCloudPreprocessorConfig_descriptor_, &PointCloudPreprocessorConfig::default_instance());
}

}  // namespace

void protobuf_ShutdownFile_pointcloud_5fpreprocessor_5fconfig_2eproto() {
  delete PointCloudPreprocessorConfig::default_instance_;
  delete PointCloudPreprocessorConfig_reflection_;
}

void protobuf_AddDesc_pointcloud_5fpreprocessor_5fconfig_2eproto() GOOGLE_ATTRIBUTE_COLD;
void protobuf_AddDesc_pointcloud_5fpreprocessor_5fconfig_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n$pointcloud_preprocessor_config.proto\022\027"
    "apollo.perception.lidar\"\222\002\n\034PointCloudPr"
    "eprocessorConfig\022\"\n\024filter_naninf_points"
    "\030\001 \001(\010:\004true\022\'\n\030filter_nearby_box_points"
    "\030\002 \001(\010:\005false\022\030\n\rbox_forward_x\030\003 \001(\002:\0010\022"
    "\031\n\016box_backward_x\030\004 \001(\002:\0010\022\030\n\rbox_forwar"
    "d_y\030\005 \001(\002:\0010\022\031\n\016box_backward_y\030\006 \001(\002:\0010\022"
    "#\n\024filter_high_z_points\030\007 \001(\010:\005false\022\026\n\013"
    "z_threshold\030\010 \001(\002:\0015", 340);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "pointcloud_preprocessor_config.proto", &protobuf_RegisterTypes);
  PointCloudPreprocessorConfig::default_instance_ = new PointCloudPreprocessorConfig();
  PointCloudPreprocessorConfig::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_pointcloud_5fpreprocessor_5fconfig_2eproto);
}

// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_pointcloud_5fpreprocessor_5fconfig_2eproto {
  StaticDescriptorInitializer_pointcloud_5fpreprocessor_5fconfig_2eproto() {
    protobuf_AddDesc_pointcloud_5fpreprocessor_5fconfig_2eproto();
  }
} static_descriptor_initializer_pointcloud_5fpreprocessor_5fconfig_2eproto_;

// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int PointCloudPreprocessorConfig::kFilterNaninfPointsFieldNumber;
const int PointCloudPreprocessorConfig::kFilterNearbyBoxPointsFieldNumber;
const int PointCloudPreprocessorConfig::kBoxForwardXFieldNumber;
const int PointCloudPreprocessorConfig::kBoxBackwardXFieldNumber;
const int PointCloudPreprocessorConfig::kBoxForwardYFieldNumber;
const int PointCloudPreprocessorConfig::kBoxBackwardYFieldNumber;
const int PointCloudPreprocessorConfig::kFilterHighZPointsFieldNumber;
const int PointCloudPreprocessorConfig::kZThresholdFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

PointCloudPreprocessorConfig::PointCloudPreprocessorConfig()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.perception.lidar.PointCloudPreprocessorConfig)
}

void PointCloudPreprocessorConfig::InitAsDefaultInstance() {
}

PointCloudPreprocessorConfig::PointCloudPreprocessorConfig(const PointCloudPreprocessorConfig& from)
  : ::google::protobuf::Message(),
    _internal_metadata_(NULL) {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:apollo.perception.lidar.PointCloudPreprocessorConfig)
}

void PointCloudPreprocessorConfig::SharedCtor() {
  _cached_size_ = 0;
  filter_naninf_points_ = true;
  filter_nearby_box_points_ = false;
  box_forward_x_ = 0;
  box_backward_x_ = 0;
  box_forward_y_ = 0;
  box_backward_y_ = 0;
  filter_high_z_points_ = false;
  z_threshold_ = 5;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

PointCloudPreprocessorConfig::~PointCloudPreprocessorConfig() {
  // @@protoc_insertion_point(destructor:apollo.perception.lidar.PointCloudPreprocessorConfig)
  SharedDtor();
}

void PointCloudPreprocessorConfig::SharedDtor() {
  if (this != default_instance_) {
  }
}

void PointCloudPreprocessorConfig::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* PointCloudPreprocessorConfig::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return PointCloudPreprocessorConfig_descriptor_;
}

const PointCloudPreprocessorConfig& PointCloudPreprocessorConfig::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_pointcloud_5fpreprocessor_5fconfig_2eproto();
  return *default_instance_;
}

PointCloudPreprocessorConfig* PointCloudPreprocessorConfig::default_instance_ = NULL;

PointCloudPreprocessorConfig* PointCloudPreprocessorConfig::New(::google::protobuf::Arena* arena) const {
  PointCloudPreprocessorConfig* n = new PointCloudPreprocessorConfig;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void PointCloudPreprocessorConfig::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.perception.lidar.PointCloudPreprocessorConfig)
#if defined(__clang__)
#define ZR_HELPER_(f) \
  _Pragma("clang diagnostic push") \
  _Pragma("clang diagnostic ignored \"-Winvalid-offsetof\"") \
  __builtin_offsetof(PointCloudPreprocessorConfig, f) \
  _Pragma("clang diagnostic pop")
#else
#define ZR_HELPER_(f) reinterpret_cast<char*>(\
  &reinterpret_cast<PointCloudPreprocessorConfig*>(16)->f)
#endif

#define ZR_(first, last) do {\
  ::memset(&first, 0,\
           ZR_HELPER_(last) - ZR_HELPER_(first) + sizeof(last));\
} while (0)

  if (_has_bits_[0 / 32] & 255u) {
    ZR_(filter_nearby_box_points_, box_backward_y_);
    filter_naninf_points_ = true;
    box_forward_x_ = 0;
    z_threshold_ = 5;
  }

#undef ZR_HELPER_
#undef ZR_

  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  if (_internal_metadata_.have_unknown_fields()) {
    mutable_unknown_fields()->Clear();
  }
}

bool PointCloudPreprocessorConfig::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.perception.lidar.PointCloudPreprocessorConfig)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional bool filter_naninf_points = 1 [default = true];
      case 1: {
        if (tag == 8) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   bool, ::google::protobuf::internal::WireFormatLite::TYPE_BOOL>(
                 input, &filter_naninf_points_)));
          set_has_filter_naninf_points();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(16)) goto parse_filter_nearby_box_points;
        break;
      }

      // optional bool filter_nearby_box_points = 2 [default = false];
      case 2: {
        if (tag == 16) {
         parse_filter_nearby_box_points:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   bool, ::google::protobuf::internal::WireFormatLite::TYPE_BOOL>(
                 input, &filter_nearby_box_points_)));
          set_has_filter_nearby_box_points();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(29)) goto parse_box_forward_x;
        break;
      }

      // optional float box_forward_x = 3 [default = 0];
      case 3: {
        if (tag == 29) {
         parse_box_forward_x:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &box_forward_x_)));
          set_has_box_forward_x();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(37)) goto parse_box_backward_x;
        break;
      }

      // optional float box_backward_x = 4 [default = 0];
      case 4: {
        if (tag == 37) {
         parse_box_backward_x:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &box_backward_x_)));
          set_has_box_backward_x();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(45)) goto parse_box_forward_y;
        break;
      }

      // optional float box_forward_y = 5 [default = 0];
      case 5: {
        if (tag == 45) {
         parse_box_forward_y:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &box_forward_y_)));
          set_has_box_forward_y();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(53)) goto parse_box_backward_y;
        break;
      }

      // optional float box_backward_y = 6 [default = 0];
      case 6: {
        if (tag == 53) {
         parse_box_backward_y:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &box_backward_y_)));
          set_has_box_backward_y();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(56)) goto parse_filter_high_z_points;
        break;
      }

      // optional bool filter_high_z_points = 7 [default = false];
      case 7: {
        if (tag == 56) {
         parse_filter_high_z_points:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   bool, ::google::protobuf::internal::WireFormatLite::TYPE_BOOL>(
                 input, &filter_high_z_points_)));
          set_has_filter_high_z_points();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(69)) goto parse_z_threshold;
        break;
      }

      // optional float z_threshold = 8 [default = 5];
      case 8: {
        if (tag == 69) {
         parse_z_threshold:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &z_threshold_)));
          set_has_z_threshold();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectAtEnd()) goto success;
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0 ||
            ::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:apollo.perception.lidar.PointCloudPreprocessorConfig)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.perception.lidar.PointCloudPreprocessorConfig)
  return false;
#undef DO_
}

void PointCloudPreprocessorConfig::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.perception.lidar.PointCloudPreprocessorConfig)
  // optional bool filter_naninf_points = 1 [default = true];
  if (has_filter_naninf_points()) {
    ::google::protobuf::internal::WireFormatLite::WriteBool(1, this->filter_naninf_points(), output);
  }

  // optional bool filter_nearby_box_points = 2 [default = false];
  if (has_filter_nearby_box_points()) {
    ::google::protobuf::internal::WireFormatLite::WriteBool(2, this->filter_nearby_box_points(), output);
  }

  // optional float box_forward_x = 3 [default = 0];
  if (has_box_forward_x()) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(3, this->box_forward_x(), output);
  }

  // optional float box_backward_x = 4 [default = 0];
  if (has_box_backward_x()) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(4, this->box_backward_x(), output);
  }

  // optional float box_forward_y = 5 [default = 0];
  if (has_box_forward_y()) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(5, this->box_forward_y(), output);
  }

  // optional float box_backward_y = 6 [default = 0];
  if (has_box_backward_y()) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(6, this->box_backward_y(), output);
  }

  // optional bool filter_high_z_points = 7 [default = false];
  if (has_filter_high_z_points()) {
    ::google::protobuf::internal::WireFormatLite::WriteBool(7, this->filter_high_z_points(), output);
  }

  // optional float z_threshold = 8 [default = 5];
  if (has_z_threshold()) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(8, this->z_threshold(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.perception.lidar.PointCloudPreprocessorConfig)
}

::google::protobuf::uint8* PointCloudPreprocessorConfig::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.perception.lidar.PointCloudPreprocessorConfig)
  // optional bool filter_naninf_points = 1 [default = true];
  if (has_filter_naninf_points()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteBoolToArray(1, this->filter_naninf_points(), target);
  }

  // optional bool filter_nearby_box_points = 2 [default = false];
  if (has_filter_nearby_box_points()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteBoolToArray(2, this->filter_nearby_box_points(), target);
  }

  // optional float box_forward_x = 3 [default = 0];
  if (has_box_forward_x()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(3, this->box_forward_x(), target);
  }

  // optional float box_backward_x = 4 [default = 0];
  if (has_box_backward_x()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(4, this->box_backward_x(), target);
  }

  // optional float box_forward_y = 5 [default = 0];
  if (has_box_forward_y()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(5, this->box_forward_y(), target);
  }

  // optional float box_backward_y = 6 [default = 0];
  if (has_box_backward_y()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(6, this->box_backward_y(), target);
  }

  // optional bool filter_high_z_points = 7 [default = false];
  if (has_filter_high_z_points()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteBoolToArray(7, this->filter_high_z_points(), target);
  }

  // optional float z_threshold = 8 [default = 5];
  if (has_z_threshold()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(8, this->z_threshold(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.perception.lidar.PointCloudPreprocessorConfig)
  return target;
}

int PointCloudPreprocessorConfig::ByteSize() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.perception.lidar.PointCloudPreprocessorConfig)
  int total_size = 0;

  if (_has_bits_[0 / 32] & 255u) {
    // optional bool filter_naninf_points = 1 [default = true];
    if (has_filter_naninf_points()) {
      total_size += 1 + 1;
    }

    // optional bool filter_nearby_box_points = 2 [default = false];
    if (has_filter_nearby_box_points()) {
      total_size += 1 + 1;
    }

    // optional float box_forward_x = 3 [default = 0];
    if (has_box_forward_x()) {
      total_size += 1 + 4;
    }

    // optional float box_backward_x = 4 [default = 0];
    if (has_box_backward_x()) {
      total_size += 1 + 4;
    }

    // optional float box_forward_y = 5 [default = 0];
    if (has_box_forward_y()) {
      total_size += 1 + 4;
    }

    // optional float box_backward_y = 6 [default = 0];
    if (has_box_backward_y()) {
      total_size += 1 + 4;
    }

    // optional bool filter_high_z_points = 7 [default = false];
    if (has_filter_high_z_points()) {
      total_size += 1 + 1;
    }

    // optional float z_threshold = 8 [default = 5];
    if (has_z_threshold()) {
      total_size += 1 + 4;
    }

  }
  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void PointCloudPreprocessorConfig::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.perception.lidar.PointCloudPreprocessorConfig)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  const PointCloudPreprocessorConfig* source = 
      ::google::protobuf::internal::DynamicCastToGenerated<const PointCloudPreprocessorConfig>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.perception.lidar.PointCloudPreprocessorConfig)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.perception.lidar.PointCloudPreprocessorConfig)
    MergeFrom(*source);
  }
}

void PointCloudPreprocessorConfig::MergeFrom(const PointCloudPreprocessorConfig& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.perception.lidar.PointCloudPreprocessorConfig)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_filter_naninf_points()) {
      set_filter_naninf_points(from.filter_naninf_points());
    }
    if (from.has_filter_nearby_box_points()) {
      set_filter_nearby_box_points(from.filter_nearby_box_points());
    }
    if (from.has_box_forward_x()) {
      set_box_forward_x(from.box_forward_x());
    }
    if (from.has_box_backward_x()) {
      set_box_backward_x(from.box_backward_x());
    }
    if (from.has_box_forward_y()) {
      set_box_forward_y(from.box_forward_y());
    }
    if (from.has_box_backward_y()) {
      set_box_backward_y(from.box_backward_y());
    }
    if (from.has_filter_high_z_points()) {
      set_filter_high_z_points(from.filter_high_z_points());
    }
    if (from.has_z_threshold()) {
      set_z_threshold(from.z_threshold());
    }
  }
  if (from._internal_metadata_.have_unknown_fields()) {
    mutable_unknown_fields()->MergeFrom(from.unknown_fields());
  }
}

void PointCloudPreprocessorConfig::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.perception.lidar.PointCloudPreprocessorConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void PointCloudPreprocessorConfig::CopyFrom(const PointCloudPreprocessorConfig& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.perception.lidar.PointCloudPreprocessorConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool PointCloudPreprocessorConfig::IsInitialized() const {

  return true;
}

void PointCloudPreprocessorConfig::Swap(PointCloudPreprocessorConfig* other) {
  if (other == this) return;
  InternalSwap(other);
}
void PointCloudPreprocessorConfig::InternalSwap(PointCloudPreprocessorConfig* other) {
  std::swap(filter_naninf_points_, other->filter_naninf_points_);
  std::swap(filter_nearby_box_points_, other->filter_nearby_box_points_);
  std::swap(box_forward_x_, other->box_forward_x_);
  std::swap(box_backward_x_, other->box_backward_x_);
  std::swap(box_forward_y_, other->box_forward_y_);
  std::swap(box_backward_y_, other->box_backward_y_);
  std::swap(filter_high_z_points_, other->filter_high_z_points_);
  std::swap(z_threshold_, other->z_threshold_);
  std::swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  std::swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata PointCloudPreprocessorConfig::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = PointCloudPreprocessorConfig_descriptor_;
  metadata.reflection = PointCloudPreprocessorConfig_reflection_;
  return metadata;
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// PointCloudPreprocessorConfig

// optional bool filter_naninf_points = 1 [default = true];
bool PointCloudPreprocessorConfig::has_filter_naninf_points() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
void PointCloudPreprocessorConfig::set_has_filter_naninf_points() {
  _has_bits_[0] |= 0x00000001u;
}
void PointCloudPreprocessorConfig::clear_has_filter_naninf_points() {
  _has_bits_[0] &= ~0x00000001u;
}
void PointCloudPreprocessorConfig::clear_filter_naninf_points() {
  filter_naninf_points_ = true;
  clear_has_filter_naninf_points();
}
 bool PointCloudPreprocessorConfig::filter_naninf_points() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.PointCloudPreprocessorConfig.filter_naninf_points)
  return filter_naninf_points_;
}
 void PointCloudPreprocessorConfig::set_filter_naninf_points(bool value) {
  set_has_filter_naninf_points();
  filter_naninf_points_ = value;
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.PointCloudPreprocessorConfig.filter_naninf_points)
}

// optional bool filter_nearby_box_points = 2 [default = false];
bool PointCloudPreprocessorConfig::has_filter_nearby_box_points() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
void PointCloudPreprocessorConfig::set_has_filter_nearby_box_points() {
  _has_bits_[0] |= 0x00000002u;
}
void PointCloudPreprocessorConfig::clear_has_filter_nearby_box_points() {
  _has_bits_[0] &= ~0x00000002u;
}
void PointCloudPreprocessorConfig::clear_filter_nearby_box_points() {
  filter_nearby_box_points_ = false;
  clear_has_filter_nearby_box_points();
}
 bool PointCloudPreprocessorConfig::filter_nearby_box_points() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.PointCloudPreprocessorConfig.filter_nearby_box_points)
  return filter_nearby_box_points_;
}
 void PointCloudPreprocessorConfig::set_filter_nearby_box_points(bool value) {
  set_has_filter_nearby_box_points();
  filter_nearby_box_points_ = value;
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.PointCloudPreprocessorConfig.filter_nearby_box_points)
}

// optional float box_forward_x = 3 [default = 0];
bool PointCloudPreprocessorConfig::has_box_forward_x() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
void PointCloudPreprocessorConfig::set_has_box_forward_x() {
  _has_bits_[0] |= 0x00000004u;
}
void PointCloudPreprocessorConfig::clear_has_box_forward_x() {
  _has_bits_[0] &= ~0x00000004u;
}
void PointCloudPreprocessorConfig::clear_box_forward_x() {
  box_forward_x_ = 0;
  clear_has_box_forward_x();
}
 float PointCloudPreprocessorConfig::box_forward_x() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.PointCloudPreprocessorConfig.box_forward_x)
  return box_forward_x_;
}
 void PointCloudPreprocessorConfig::set_box_forward_x(float value) {
  set_has_box_forward_x();
  box_forward_x_ = value;
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.PointCloudPreprocessorConfig.box_forward_x)
}

// optional float box_backward_x = 4 [default = 0];
bool PointCloudPreprocessorConfig::has_box_backward_x() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
void PointCloudPreprocessorConfig::set_has_box_backward_x() {
  _has_bits_[0] |= 0x00000008u;
}
void PointCloudPreprocessorConfig::clear_has_box_backward_x() {
  _has_bits_[0] &= ~0x00000008u;
}
void PointCloudPreprocessorConfig::clear_box_backward_x() {
  box_backward_x_ = 0;
  clear_has_box_backward_x();
}
 float PointCloudPreprocessorConfig::box_backward_x() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.PointCloudPreprocessorConfig.box_backward_x)
  return box_backward_x_;
}
 void PointCloudPreprocessorConfig::set_box_backward_x(float value) {
  set_has_box_backward_x();
  box_backward_x_ = value;
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.PointCloudPreprocessorConfig.box_backward_x)
}

// optional float box_forward_y = 5 [default = 0];
bool PointCloudPreprocessorConfig::has_box_forward_y() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
void PointCloudPreprocessorConfig::set_has_box_forward_y() {
  _has_bits_[0] |= 0x00000010u;
}
void PointCloudPreprocessorConfig::clear_has_box_forward_y() {
  _has_bits_[0] &= ~0x00000010u;
}
void PointCloudPreprocessorConfig::clear_box_forward_y() {
  box_forward_y_ = 0;
  clear_has_box_forward_y();
}
 float PointCloudPreprocessorConfig::box_forward_y() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.PointCloudPreprocessorConfig.box_forward_y)
  return box_forward_y_;
}
 void PointCloudPreprocessorConfig::set_box_forward_y(float value) {
  set_has_box_forward_y();
  box_forward_y_ = value;
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.PointCloudPreprocessorConfig.box_forward_y)
}

// optional float box_backward_y = 6 [default = 0];
bool PointCloudPreprocessorConfig::has_box_backward_y() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
void PointCloudPreprocessorConfig::set_has_box_backward_y() {
  _has_bits_[0] |= 0x00000020u;
}
void PointCloudPreprocessorConfig::clear_has_box_backward_y() {
  _has_bits_[0] &= ~0x00000020u;
}
void PointCloudPreprocessorConfig::clear_box_backward_y() {
  box_backward_y_ = 0;
  clear_has_box_backward_y();
}
 float PointCloudPreprocessorConfig::box_backward_y() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.PointCloudPreprocessorConfig.box_backward_y)
  return box_backward_y_;
}
 void PointCloudPreprocessorConfig::set_box_backward_y(float value) {
  set_has_box_backward_y();
  box_backward_y_ = value;
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.PointCloudPreprocessorConfig.box_backward_y)
}

// optional bool filter_high_z_points = 7 [default = false];
bool PointCloudPreprocessorConfig::has_filter_high_z_points() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
void PointCloudPreprocessorConfig::set_has_filter_high_z_points() {
  _has_bits_[0] |= 0x00000040u;
}
void PointCloudPreprocessorConfig::clear_has_filter_high_z_points() {
  _has_bits_[0] &= ~0x00000040u;
}
void PointCloudPreprocessorConfig::clear_filter_high_z_points() {
  filter_high_z_points_ = false;
  clear_has_filter_high_z_points();
}
 bool PointCloudPreprocessorConfig::filter_high_z_points() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.PointCloudPreprocessorConfig.filter_high_z_points)
  return filter_high_z_points_;
}
 void PointCloudPreprocessorConfig::set_filter_high_z_points(bool value) {
  set_has_filter_high_z_points();
  filter_high_z_points_ = value;
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.PointCloudPreprocessorConfig.filter_high_z_points)
}

// optional float z_threshold = 8 [default = 5];
bool PointCloudPreprocessorConfig::has_z_threshold() const {
  return (_has_bits_[0] & 0x00000080u) != 0;
}
void PointCloudPreprocessorConfig::set_has_z_threshold() {
  _has_bits_[0] |= 0x00000080u;
}
void PointCloudPreprocessorConfig::clear_has_z_threshold() {
  _has_bits_[0] &= ~0x00000080u;
}
void PointCloudPreprocessorConfig::clear_z_threshold() {
  z_threshold_ = 5;
  clear_has_z_threshold();
}
 float PointCloudPreprocessorConfig::z_threshold() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.PointCloudPreprocessorConfig.z_threshold)
  return z_threshold_;
}
 void PointCloudPreprocessorConfig::set_z_threshold(float value) {
  set_has_z_threshold();
  z_threshold_ = value;
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.PointCloudPreprocessorConfig.z_threshold)
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo

// @@protoc_insertion_point(global_scope)