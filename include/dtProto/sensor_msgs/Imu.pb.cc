// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: dtProto/sensor_msgs/Imu.proto

#include "dtProto/sensor_msgs/Imu.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>

PROTOBUF_PRAGMA_INIT_SEG

namespace _pb = ::PROTOBUF_NAMESPACE_ID;
namespace _pbi = _pb::internal;

namespace dtproto {
namespace sensor_msgs {
PROTOBUF_CONSTEXPR Imu::Imu(
    ::_pbi::ConstantInitialized): _impl_{
    /*decltype(_impl_.orientaton_)*/nullptr
  , /*decltype(_impl_.angular_velocity_)*/nullptr
  , /*decltype(_impl_.linear_acceleration_)*/nullptr
  , /*decltype(_impl_._cached_size_)*/{}} {}
struct ImuDefaultTypeInternal {
  PROTOBUF_CONSTEXPR ImuDefaultTypeInternal()
      : _instance(::_pbi::ConstantInitialized{}) {}
  ~ImuDefaultTypeInternal() {}
  union {
    Imu _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 ImuDefaultTypeInternal _Imu_default_instance_;
}  // namespace sensor_msgs
}  // namespace dtproto
static ::_pb::Metadata file_level_metadata_dtProto_2fsensor_5fmsgs_2fImu_2eproto[1];
static constexpr ::_pb::EnumDescriptor const** file_level_enum_descriptors_dtProto_2fsensor_5fmsgs_2fImu_2eproto = nullptr;
static constexpr ::_pb::ServiceDescriptor const** file_level_service_descriptors_dtProto_2fsensor_5fmsgs_2fImu_2eproto = nullptr;

const uint32_t TableStruct_dtProto_2fsensor_5fmsgs_2fImu_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::dtproto::sensor_msgs::Imu, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::dtproto::sensor_msgs::Imu, _impl_.orientaton_),
  PROTOBUF_FIELD_OFFSET(::dtproto::sensor_msgs::Imu, _impl_.angular_velocity_),
  PROTOBUF_FIELD_OFFSET(::dtproto::sensor_msgs::Imu, _impl_.linear_acceleration_),
};
static const ::_pbi::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::dtproto::sensor_msgs::Imu)},
};

static const ::_pb::Message* const file_default_instances[] = {
  &::dtproto::sensor_msgs::_Imu_default_instance_._instance,
};

const char descriptor_table_protodef_dtProto_2fsensor_5fmsgs_2fImu_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\035dtProto/sensor_msgs/Imu.proto\022\023dtproto"
  ".sensor_msgs\032\"dtProto/geometry_msgs/Vect"
  "or.proto\032\'dtProto/geometry_msgs/Orientat"
  "ion.proto\"\265\001\n\003Imu\0225\n\norientaton\030\001 \001(\0132!."
  "dtproto.geometry_msgs.Quaternion\0229\n\020angu"
  "lar_velocity\030\002 \001(\0132\037.dtproto.geometry_ms"
  "gs.Vector3d\022<\n\023linear_acceleration\030\003 \001(\013"
  "2\037.dtproto.geometry_msgs.Vector3db\006proto"
  "3"
  ;
static const ::_pbi::DescriptorTable* const descriptor_table_dtProto_2fsensor_5fmsgs_2fImu_2eproto_deps[2] = {
  &::descriptor_table_dtProto_2fgeometry_5fmsgs_2fOrientation_2eproto,
  &::descriptor_table_dtProto_2fgeometry_5fmsgs_2fVector_2eproto,
};
static ::_pbi::once_flag descriptor_table_dtProto_2fsensor_5fmsgs_2fImu_2eproto_once;
const ::_pbi::DescriptorTable descriptor_table_dtProto_2fsensor_5fmsgs_2fImu_2eproto = {
    false, false, 321, descriptor_table_protodef_dtProto_2fsensor_5fmsgs_2fImu_2eproto,
    "dtProto/sensor_msgs/Imu.proto",
    &descriptor_table_dtProto_2fsensor_5fmsgs_2fImu_2eproto_once, descriptor_table_dtProto_2fsensor_5fmsgs_2fImu_2eproto_deps, 2, 1,
    schemas, file_default_instances, TableStruct_dtProto_2fsensor_5fmsgs_2fImu_2eproto::offsets,
    file_level_metadata_dtProto_2fsensor_5fmsgs_2fImu_2eproto, file_level_enum_descriptors_dtProto_2fsensor_5fmsgs_2fImu_2eproto,
    file_level_service_descriptors_dtProto_2fsensor_5fmsgs_2fImu_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::_pbi::DescriptorTable* descriptor_table_dtProto_2fsensor_5fmsgs_2fImu_2eproto_getter() {
  return &descriptor_table_dtProto_2fsensor_5fmsgs_2fImu_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY2 static ::_pbi::AddDescriptorsRunner dynamic_init_dummy_dtProto_2fsensor_5fmsgs_2fImu_2eproto(&descriptor_table_dtProto_2fsensor_5fmsgs_2fImu_2eproto);
namespace dtproto {
namespace sensor_msgs {

// ===================================================================

class Imu::_Internal {
 public:
  static const ::dtproto::geometry_msgs::Quaternion& orientaton(const Imu* msg);
  static const ::dtproto::geometry_msgs::Vector3d& angular_velocity(const Imu* msg);
  static const ::dtproto::geometry_msgs::Vector3d& linear_acceleration(const Imu* msg);
};

const ::dtproto::geometry_msgs::Quaternion&
Imu::_Internal::orientaton(const Imu* msg) {
  return *msg->_impl_.orientaton_;
}
const ::dtproto::geometry_msgs::Vector3d&
Imu::_Internal::angular_velocity(const Imu* msg) {
  return *msg->_impl_.angular_velocity_;
}
const ::dtproto::geometry_msgs::Vector3d&
Imu::_Internal::linear_acceleration(const Imu* msg) {
  return *msg->_impl_.linear_acceleration_;
}
void Imu::clear_orientaton() {
  if (GetArenaForAllocation() == nullptr && _impl_.orientaton_ != nullptr) {
    delete _impl_.orientaton_;
  }
  _impl_.orientaton_ = nullptr;
}
void Imu::clear_angular_velocity() {
  if (GetArenaForAllocation() == nullptr && _impl_.angular_velocity_ != nullptr) {
    delete _impl_.angular_velocity_;
  }
  _impl_.angular_velocity_ = nullptr;
}
void Imu::clear_linear_acceleration() {
  if (GetArenaForAllocation() == nullptr && _impl_.linear_acceleration_ != nullptr) {
    delete _impl_.linear_acceleration_;
  }
  _impl_.linear_acceleration_ = nullptr;
}
Imu::Imu(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor(arena, is_message_owned);
  // @@protoc_insertion_point(arena_constructor:dtproto.sensor_msgs.Imu)
}
Imu::Imu(const Imu& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  Imu* const _this = this; (void)_this;
  new (&_impl_) Impl_{
      decltype(_impl_.orientaton_){nullptr}
    , decltype(_impl_.angular_velocity_){nullptr}
    , decltype(_impl_.linear_acceleration_){nullptr}
    , /*decltype(_impl_._cached_size_)*/{}};

  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_orientaton()) {
    _this->_impl_.orientaton_ = new ::dtproto::geometry_msgs::Quaternion(*from._impl_.orientaton_);
  }
  if (from._internal_has_angular_velocity()) {
    _this->_impl_.angular_velocity_ = new ::dtproto::geometry_msgs::Vector3d(*from._impl_.angular_velocity_);
  }
  if (from._internal_has_linear_acceleration()) {
    _this->_impl_.linear_acceleration_ = new ::dtproto::geometry_msgs::Vector3d(*from._impl_.linear_acceleration_);
  }
  // @@protoc_insertion_point(copy_constructor:dtproto.sensor_msgs.Imu)
}

inline void Imu::SharedCtor(
    ::_pb::Arena* arena, bool is_message_owned) {
  (void)arena;
  (void)is_message_owned;
  new (&_impl_) Impl_{
      decltype(_impl_.orientaton_){nullptr}
    , decltype(_impl_.angular_velocity_){nullptr}
    , decltype(_impl_.linear_acceleration_){nullptr}
    , /*decltype(_impl_._cached_size_)*/{}
  };
}

Imu::~Imu() {
  // @@protoc_insertion_point(destructor:dtproto.sensor_msgs.Imu)
  if (auto *arena = _internal_metadata_.DeleteReturnArena<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>()) {
  (void)arena;
    return;
  }
  SharedDtor();
}

inline void Imu::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete _impl_.orientaton_;
  if (this != internal_default_instance()) delete _impl_.angular_velocity_;
  if (this != internal_default_instance()) delete _impl_.linear_acceleration_;
}

void Imu::SetCachedSize(int size) const {
  _impl_._cached_size_.Set(size);
}

void Imu::Clear() {
// @@protoc_insertion_point(message_clear_start:dtproto.sensor_msgs.Imu)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  if (GetArenaForAllocation() == nullptr && _impl_.orientaton_ != nullptr) {
    delete _impl_.orientaton_;
  }
  _impl_.orientaton_ = nullptr;
  if (GetArenaForAllocation() == nullptr && _impl_.angular_velocity_ != nullptr) {
    delete _impl_.angular_velocity_;
  }
  _impl_.angular_velocity_ = nullptr;
  if (GetArenaForAllocation() == nullptr && _impl_.linear_acceleration_ != nullptr) {
    delete _impl_.linear_acceleration_;
  }
  _impl_.linear_acceleration_ = nullptr;
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* Imu::_InternalParse(const char* ptr, ::_pbi::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::_pbi::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // .dtproto.geometry_msgs.Quaternion orientaton = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_orientaton(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // .dtproto.geometry_msgs.Vector3d angular_velocity = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 18)) {
          ptr = ctx->ParseMessage(_internal_mutable_angular_velocity(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // .dtproto.geometry_msgs.Vector3d linear_acceleration = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 26)) {
          ptr = ctx->ParseMessage(_internal_mutable_linear_acceleration(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      default:
        goto handle_unusual;
    }  // switch
  handle_unusual:
    if ((tag == 0) || ((tag & 7) == 4)) {
      CHK_(ptr);
      ctx->SetLastTag(tag);
      goto message_done;
    }
    ptr = UnknownFieldParse(
        tag,
        _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
        ptr, ctx);
    CHK_(ptr != nullptr);
  }  // while
message_done:
  return ptr;
failure:
  ptr = nullptr;
  goto message_done;
#undef CHK_
}

uint8_t* Imu::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:dtproto.sensor_msgs.Imu)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // .dtproto.geometry_msgs.Quaternion orientaton = 1;
  if (this->_internal_has_orientaton()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(1, _Internal::orientaton(this),
        _Internal::orientaton(this).GetCachedSize(), target, stream);
  }

  // .dtproto.geometry_msgs.Vector3d angular_velocity = 2;
  if (this->_internal_has_angular_velocity()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(2, _Internal::angular_velocity(this),
        _Internal::angular_velocity(this).GetCachedSize(), target, stream);
  }

  // .dtproto.geometry_msgs.Vector3d linear_acceleration = 3;
  if (this->_internal_has_linear_acceleration()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(3, _Internal::linear_acceleration(this),
        _Internal::linear_acceleration(this).GetCachedSize(), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:dtproto.sensor_msgs.Imu)
  return target;
}

size_t Imu::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:dtproto.sensor_msgs.Imu)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // .dtproto.geometry_msgs.Quaternion orientaton = 1;
  if (this->_internal_has_orientaton()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *_impl_.orientaton_);
  }

  // .dtproto.geometry_msgs.Vector3d angular_velocity = 2;
  if (this->_internal_has_angular_velocity()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *_impl_.angular_velocity_);
  }

  // .dtproto.geometry_msgs.Vector3d linear_acceleration = 3;
  if (this->_internal_has_linear_acceleration()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *_impl_.linear_acceleration_);
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_impl_._cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData Imu::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSourceCheck,
    Imu::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*Imu::GetClassData() const { return &_class_data_; }


void Imu::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg) {
  auto* const _this = static_cast<Imu*>(&to_msg);
  auto& from = static_cast<const Imu&>(from_msg);
  // @@protoc_insertion_point(class_specific_merge_from_start:dtproto.sensor_msgs.Imu)
  GOOGLE_DCHECK_NE(&from, _this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  if (from._internal_has_orientaton()) {
    _this->_internal_mutable_orientaton()->::dtproto::geometry_msgs::Quaternion::MergeFrom(
        from._internal_orientaton());
  }
  if (from._internal_has_angular_velocity()) {
    _this->_internal_mutable_angular_velocity()->::dtproto::geometry_msgs::Vector3d::MergeFrom(
        from._internal_angular_velocity());
  }
  if (from._internal_has_linear_acceleration()) {
    _this->_internal_mutable_linear_acceleration()->::dtproto::geometry_msgs::Vector3d::MergeFrom(
        from._internal_linear_acceleration());
  }
  _this->_internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void Imu::CopyFrom(const Imu& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:dtproto.sensor_msgs.Imu)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Imu::IsInitialized() const {
  return true;
}

void Imu::InternalSwap(Imu* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(Imu, _impl_.linear_acceleration_)
      + sizeof(Imu::_impl_.linear_acceleration_)
      - PROTOBUF_FIELD_OFFSET(Imu, _impl_.orientaton_)>(
          reinterpret_cast<char*>(&_impl_.orientaton_),
          reinterpret_cast<char*>(&other->_impl_.orientaton_));
}

::PROTOBUF_NAMESPACE_ID::Metadata Imu::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_dtProto_2fsensor_5fmsgs_2fImu_2eproto_getter, &descriptor_table_dtProto_2fsensor_5fmsgs_2fImu_2eproto_once,
      file_level_metadata_dtProto_2fsensor_5fmsgs_2fImu_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace sensor_msgs
}  // namespace dtproto
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::dtproto::sensor_msgs::Imu*
Arena::CreateMaybeMessage< ::dtproto::sensor_msgs::Imu >(Arena* arena) {
  return Arena::CreateMessageInternal< ::dtproto::sensor_msgs::Imu >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>