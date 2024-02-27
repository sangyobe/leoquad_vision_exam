// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: dtProto/sensor_msgs/BatteryState.proto

#include "dtProto/sensor_msgs/BatteryState.pb.h"

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
PROTOBUF_CONSTEXPR BatteryState::BatteryState(
    ::_pbi::ConstantInitialized): _impl_{
    /*decltype(_impl_.voltage_)*/0
  , /*decltype(_impl_.temperature_)*/0
  , /*decltype(_impl_.current_)*/0
  , /*decltype(_impl_.charge_)*/0
  , /*decltype(_impl_.design_capacity_)*/0
  , /*decltype(_impl_.percentage_)*/0
  , /*decltype(_impl_.charging_status_)*/0
  , /*decltype(_impl_._cached_size_)*/{}} {}
struct BatteryStateDefaultTypeInternal {
  PROTOBUF_CONSTEXPR BatteryStateDefaultTypeInternal()
      : _instance(::_pbi::ConstantInitialized{}) {}
  ~BatteryStateDefaultTypeInternal() {}
  union {
    BatteryState _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 BatteryStateDefaultTypeInternal _BatteryState_default_instance_;
}  // namespace sensor_msgs
}  // namespace dtproto
static ::_pb::Metadata file_level_metadata_dtProto_2fsensor_5fmsgs_2fBatteryState_2eproto[1];
static const ::_pb::EnumDescriptor* file_level_enum_descriptors_dtProto_2fsensor_5fmsgs_2fBatteryState_2eproto[1];
static constexpr ::_pb::ServiceDescriptor const** file_level_service_descriptors_dtProto_2fsensor_5fmsgs_2fBatteryState_2eproto = nullptr;

const uint32_t TableStruct_dtProto_2fsensor_5fmsgs_2fBatteryState_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::dtproto::sensor_msgs::BatteryState, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::dtproto::sensor_msgs::BatteryState, _impl_.voltage_),
  PROTOBUF_FIELD_OFFSET(::dtproto::sensor_msgs::BatteryState, _impl_.temperature_),
  PROTOBUF_FIELD_OFFSET(::dtproto::sensor_msgs::BatteryState, _impl_.current_),
  PROTOBUF_FIELD_OFFSET(::dtproto::sensor_msgs::BatteryState, _impl_.charge_),
  PROTOBUF_FIELD_OFFSET(::dtproto::sensor_msgs::BatteryState, _impl_.design_capacity_),
  PROTOBUF_FIELD_OFFSET(::dtproto::sensor_msgs::BatteryState, _impl_.percentage_),
  PROTOBUF_FIELD_OFFSET(::dtproto::sensor_msgs::BatteryState, _impl_.charging_status_),
};
static const ::_pbi::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::dtproto::sensor_msgs::BatteryState)},
};

static const ::_pb::Message* const file_default_instances[] = {
  &::dtproto::sensor_msgs::_BatteryState_default_instance_._instance,
};

const char descriptor_table_protodef_dtProto_2fsensor_5fmsgs_2fBatteryState_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n&dtProto/sensor_msgs/BatteryState.proto"
  "\022\023dtproto.sensor_msgs\"\360\001\n\014BatteryState\022\017"
  "\n\007voltage\030\001 \001(\002\022\023\n\013temperature\030\002 \001(\002\022\017\n\007"
  "current\030\003 \001(\002\022\016\n\006charge\030\004 \001(\002\022\027\n\017design_"
  "capacity\030\005 \001(\002\022\022\n\npercentage\030\006 \001(\002\022I\n\017ch"
  "arging_status\030\007 \001(\01620.dtproto.sensor_msg"
  "s.BatteryState.ChargingStatus\"!\n\016Chargin"
  "gStatus\022\006\n\002ON\020\000\022\007\n\003OFF\020\001b\006proto3"
  ;
static ::_pbi::once_flag descriptor_table_dtProto_2fsensor_5fmsgs_2fBatteryState_2eproto_once;
const ::_pbi::DescriptorTable descriptor_table_dtProto_2fsensor_5fmsgs_2fBatteryState_2eproto = {
    false, false, 312, descriptor_table_protodef_dtProto_2fsensor_5fmsgs_2fBatteryState_2eproto,
    "dtProto/sensor_msgs/BatteryState.proto",
    &descriptor_table_dtProto_2fsensor_5fmsgs_2fBatteryState_2eproto_once, nullptr, 0, 1,
    schemas, file_default_instances, TableStruct_dtProto_2fsensor_5fmsgs_2fBatteryState_2eproto::offsets,
    file_level_metadata_dtProto_2fsensor_5fmsgs_2fBatteryState_2eproto, file_level_enum_descriptors_dtProto_2fsensor_5fmsgs_2fBatteryState_2eproto,
    file_level_service_descriptors_dtProto_2fsensor_5fmsgs_2fBatteryState_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::_pbi::DescriptorTable* descriptor_table_dtProto_2fsensor_5fmsgs_2fBatteryState_2eproto_getter() {
  return &descriptor_table_dtProto_2fsensor_5fmsgs_2fBatteryState_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY2 static ::_pbi::AddDescriptorsRunner dynamic_init_dummy_dtProto_2fsensor_5fmsgs_2fBatteryState_2eproto(&descriptor_table_dtProto_2fsensor_5fmsgs_2fBatteryState_2eproto);
namespace dtproto {
namespace sensor_msgs {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* BatteryState_ChargingStatus_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_dtProto_2fsensor_5fmsgs_2fBatteryState_2eproto);
  return file_level_enum_descriptors_dtProto_2fsensor_5fmsgs_2fBatteryState_2eproto[0];
}
bool BatteryState_ChargingStatus_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
      return true;
    default:
      return false;
  }
}

#if (__cplusplus < 201703) && (!defined(_MSC_VER) || (_MSC_VER >= 1900 && _MSC_VER < 1912))
constexpr BatteryState_ChargingStatus BatteryState::ON;
constexpr BatteryState_ChargingStatus BatteryState::OFF;
constexpr BatteryState_ChargingStatus BatteryState::ChargingStatus_MIN;
constexpr BatteryState_ChargingStatus BatteryState::ChargingStatus_MAX;
constexpr int BatteryState::ChargingStatus_ARRAYSIZE;
#endif  // (__cplusplus < 201703) && (!defined(_MSC_VER) || (_MSC_VER >= 1900 && _MSC_VER < 1912))

// ===================================================================

class BatteryState::_Internal {
 public:
};

BatteryState::BatteryState(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor(arena, is_message_owned);
  // @@protoc_insertion_point(arena_constructor:dtproto.sensor_msgs.BatteryState)
}
BatteryState::BatteryState(const BatteryState& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  BatteryState* const _this = this; (void)_this;
  new (&_impl_) Impl_{
      decltype(_impl_.voltage_){}
    , decltype(_impl_.temperature_){}
    , decltype(_impl_.current_){}
    , decltype(_impl_.charge_){}
    , decltype(_impl_.design_capacity_){}
    , decltype(_impl_.percentage_){}
    , decltype(_impl_.charging_status_){}
    , /*decltype(_impl_._cached_size_)*/{}};

  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::memcpy(&_impl_.voltage_, &from._impl_.voltage_,
    static_cast<size_t>(reinterpret_cast<char*>(&_impl_.charging_status_) -
    reinterpret_cast<char*>(&_impl_.voltage_)) + sizeof(_impl_.charging_status_));
  // @@protoc_insertion_point(copy_constructor:dtproto.sensor_msgs.BatteryState)
}

inline void BatteryState::SharedCtor(
    ::_pb::Arena* arena, bool is_message_owned) {
  (void)arena;
  (void)is_message_owned;
  new (&_impl_) Impl_{
      decltype(_impl_.voltage_){0}
    , decltype(_impl_.temperature_){0}
    , decltype(_impl_.current_){0}
    , decltype(_impl_.charge_){0}
    , decltype(_impl_.design_capacity_){0}
    , decltype(_impl_.percentage_){0}
    , decltype(_impl_.charging_status_){0}
    , /*decltype(_impl_._cached_size_)*/{}
  };
}

BatteryState::~BatteryState() {
  // @@protoc_insertion_point(destructor:dtproto.sensor_msgs.BatteryState)
  if (auto *arena = _internal_metadata_.DeleteReturnArena<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>()) {
  (void)arena;
    return;
  }
  SharedDtor();
}

inline void BatteryState::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
}

void BatteryState::SetCachedSize(int size) const {
  _impl_._cached_size_.Set(size);
}

void BatteryState::Clear() {
// @@protoc_insertion_point(message_clear_start:dtproto.sensor_msgs.BatteryState)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ::memset(&_impl_.voltage_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&_impl_.charging_status_) -
      reinterpret_cast<char*>(&_impl_.voltage_)) + sizeof(_impl_.charging_status_));
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* BatteryState::_InternalParse(const char* ptr, ::_pbi::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::_pbi::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // float voltage = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 13)) {
          _impl_.voltage_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else
          goto handle_unusual;
        continue;
      // float temperature = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 21)) {
          _impl_.temperature_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else
          goto handle_unusual;
        continue;
      // float current = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 29)) {
          _impl_.current_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else
          goto handle_unusual;
        continue;
      // float charge = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 37)) {
          _impl_.charge_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else
          goto handle_unusual;
        continue;
      // float design_capacity = 5;
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 45)) {
          _impl_.design_capacity_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else
          goto handle_unusual;
        continue;
      // float percentage = 6;
      case 6:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 53)) {
          _impl_.percentage_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else
          goto handle_unusual;
        continue;
      // .dtproto.sensor_msgs.BatteryState.ChargingStatus charging_status = 7;
      case 7:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 56)) {
          uint64_t val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
          _internal_set_charging_status(static_cast<::dtproto::sensor_msgs::BatteryState_ChargingStatus>(val));
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

uint8_t* BatteryState::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:dtproto.sensor_msgs.BatteryState)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // float voltage = 1;
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_voltage = this->_internal_voltage();
  uint32_t raw_voltage;
  memcpy(&raw_voltage, &tmp_voltage, sizeof(tmp_voltage));
  if (raw_voltage != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteFloatToArray(1, this->_internal_voltage(), target);
  }

  // float temperature = 2;
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_temperature = this->_internal_temperature();
  uint32_t raw_temperature;
  memcpy(&raw_temperature, &tmp_temperature, sizeof(tmp_temperature));
  if (raw_temperature != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteFloatToArray(2, this->_internal_temperature(), target);
  }

  // float current = 3;
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_current = this->_internal_current();
  uint32_t raw_current;
  memcpy(&raw_current, &tmp_current, sizeof(tmp_current));
  if (raw_current != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteFloatToArray(3, this->_internal_current(), target);
  }

  // float charge = 4;
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_charge = this->_internal_charge();
  uint32_t raw_charge;
  memcpy(&raw_charge, &tmp_charge, sizeof(tmp_charge));
  if (raw_charge != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteFloatToArray(4, this->_internal_charge(), target);
  }

  // float design_capacity = 5;
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_design_capacity = this->_internal_design_capacity();
  uint32_t raw_design_capacity;
  memcpy(&raw_design_capacity, &tmp_design_capacity, sizeof(tmp_design_capacity));
  if (raw_design_capacity != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteFloatToArray(5, this->_internal_design_capacity(), target);
  }

  // float percentage = 6;
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_percentage = this->_internal_percentage();
  uint32_t raw_percentage;
  memcpy(&raw_percentage, &tmp_percentage, sizeof(tmp_percentage));
  if (raw_percentage != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteFloatToArray(6, this->_internal_percentage(), target);
  }

  // .dtproto.sensor_msgs.BatteryState.ChargingStatus charging_status = 7;
  if (this->_internal_charging_status() != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteEnumToArray(
      7, this->_internal_charging_status(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:dtproto.sensor_msgs.BatteryState)
  return target;
}

size_t BatteryState::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:dtproto.sensor_msgs.BatteryState)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // float voltage = 1;
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_voltage = this->_internal_voltage();
  uint32_t raw_voltage;
  memcpy(&raw_voltage, &tmp_voltage, sizeof(tmp_voltage));
  if (raw_voltage != 0) {
    total_size += 1 + 4;
  }

  // float temperature = 2;
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_temperature = this->_internal_temperature();
  uint32_t raw_temperature;
  memcpy(&raw_temperature, &tmp_temperature, sizeof(tmp_temperature));
  if (raw_temperature != 0) {
    total_size += 1 + 4;
  }

  // float current = 3;
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_current = this->_internal_current();
  uint32_t raw_current;
  memcpy(&raw_current, &tmp_current, sizeof(tmp_current));
  if (raw_current != 0) {
    total_size += 1 + 4;
  }

  // float charge = 4;
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_charge = this->_internal_charge();
  uint32_t raw_charge;
  memcpy(&raw_charge, &tmp_charge, sizeof(tmp_charge));
  if (raw_charge != 0) {
    total_size += 1 + 4;
  }

  // float design_capacity = 5;
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_design_capacity = this->_internal_design_capacity();
  uint32_t raw_design_capacity;
  memcpy(&raw_design_capacity, &tmp_design_capacity, sizeof(tmp_design_capacity));
  if (raw_design_capacity != 0) {
    total_size += 1 + 4;
  }

  // float percentage = 6;
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_percentage = this->_internal_percentage();
  uint32_t raw_percentage;
  memcpy(&raw_percentage, &tmp_percentage, sizeof(tmp_percentage));
  if (raw_percentage != 0) {
    total_size += 1 + 4;
  }

  // .dtproto.sensor_msgs.BatteryState.ChargingStatus charging_status = 7;
  if (this->_internal_charging_status() != 0) {
    total_size += 1 +
      ::_pbi::WireFormatLite::EnumSize(this->_internal_charging_status());
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_impl_._cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData BatteryState::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSourceCheck,
    BatteryState::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*BatteryState::GetClassData() const { return &_class_data_; }


void BatteryState::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg) {
  auto* const _this = static_cast<BatteryState*>(&to_msg);
  auto& from = static_cast<const BatteryState&>(from_msg);
  // @@protoc_insertion_point(class_specific_merge_from_start:dtproto.sensor_msgs.BatteryState)
  GOOGLE_DCHECK_NE(&from, _this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_voltage = from._internal_voltage();
  uint32_t raw_voltage;
  memcpy(&raw_voltage, &tmp_voltage, sizeof(tmp_voltage));
  if (raw_voltage != 0) {
    _this->_internal_set_voltage(from._internal_voltage());
  }
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_temperature = from._internal_temperature();
  uint32_t raw_temperature;
  memcpy(&raw_temperature, &tmp_temperature, sizeof(tmp_temperature));
  if (raw_temperature != 0) {
    _this->_internal_set_temperature(from._internal_temperature());
  }
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_current = from._internal_current();
  uint32_t raw_current;
  memcpy(&raw_current, &tmp_current, sizeof(tmp_current));
  if (raw_current != 0) {
    _this->_internal_set_current(from._internal_current());
  }
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_charge = from._internal_charge();
  uint32_t raw_charge;
  memcpy(&raw_charge, &tmp_charge, sizeof(tmp_charge));
  if (raw_charge != 0) {
    _this->_internal_set_charge(from._internal_charge());
  }
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_design_capacity = from._internal_design_capacity();
  uint32_t raw_design_capacity;
  memcpy(&raw_design_capacity, &tmp_design_capacity, sizeof(tmp_design_capacity));
  if (raw_design_capacity != 0) {
    _this->_internal_set_design_capacity(from._internal_design_capacity());
  }
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_percentage = from._internal_percentage();
  uint32_t raw_percentage;
  memcpy(&raw_percentage, &tmp_percentage, sizeof(tmp_percentage));
  if (raw_percentage != 0) {
    _this->_internal_set_percentage(from._internal_percentage());
  }
  if (from._internal_charging_status() != 0) {
    _this->_internal_set_charging_status(from._internal_charging_status());
  }
  _this->_internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void BatteryState::CopyFrom(const BatteryState& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:dtproto.sensor_msgs.BatteryState)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool BatteryState::IsInitialized() const {
  return true;
}

void BatteryState::InternalSwap(BatteryState* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(BatteryState, _impl_.charging_status_)
      + sizeof(BatteryState::_impl_.charging_status_)
      - PROTOBUF_FIELD_OFFSET(BatteryState, _impl_.voltage_)>(
          reinterpret_cast<char*>(&_impl_.voltage_),
          reinterpret_cast<char*>(&other->_impl_.voltage_));
}

::PROTOBUF_NAMESPACE_ID::Metadata BatteryState::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_dtProto_2fsensor_5fmsgs_2fBatteryState_2eproto_getter, &descriptor_table_dtProto_2fsensor_5fmsgs_2fBatteryState_2eproto_once,
      file_level_metadata_dtProto_2fsensor_5fmsgs_2fBatteryState_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace sensor_msgs
}  // namespace dtproto
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::dtproto::sensor_msgs::BatteryState*
Arena::CreateMaybeMessage< ::dtproto::sensor_msgs::BatteryState >(Arena* arena) {
  return Arena::CreateMessageInternal< ::dtproto::sensor_msgs::BatteryState >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>