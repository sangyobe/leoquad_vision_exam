// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: dtProto/geometry_msgs/Displacement.proto

#include "dtProto/geometry_msgs/Displacement.pb.h"

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
namespace geometry_msgs {
PROTOBUF_CONSTEXPR Displacement::Displacement(
    ::_pbi::ConstantInitialized): _impl_{
    /*decltype(_impl_.a1_)*/0
  , /*decltype(_impl_.a2_)*/0
  , /*decltype(_impl_.a3_)*/0
  , /*decltype(_impl_._cached_size_)*/{}} {}
struct DisplacementDefaultTypeInternal {
  PROTOBUF_CONSTEXPR DisplacementDefaultTypeInternal()
      : _instance(::_pbi::ConstantInitialized{}) {}
  ~DisplacementDefaultTypeInternal() {}
  union {
    Displacement _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 DisplacementDefaultTypeInternal _Displacement_default_instance_;
}  // namespace geometry_msgs
}  // namespace dtproto
static ::_pb::Metadata file_level_metadata_dtProto_2fgeometry_5fmsgs_2fDisplacement_2eproto[1];
static constexpr ::_pb::EnumDescriptor const** file_level_enum_descriptors_dtProto_2fgeometry_5fmsgs_2fDisplacement_2eproto = nullptr;
static constexpr ::_pb::ServiceDescriptor const** file_level_service_descriptors_dtProto_2fgeometry_5fmsgs_2fDisplacement_2eproto = nullptr;

const uint32_t TableStruct_dtProto_2fgeometry_5fmsgs_2fDisplacement_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::dtproto::geometry_msgs::Displacement, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::dtproto::geometry_msgs::Displacement, _impl_.a1_),
  PROTOBUF_FIELD_OFFSET(::dtproto::geometry_msgs::Displacement, _impl_.a2_),
  PROTOBUF_FIELD_OFFSET(::dtproto::geometry_msgs::Displacement, _impl_.a3_),
};
static const ::_pbi::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::dtproto::geometry_msgs::Displacement)},
};

static const ::_pb::Message* const file_default_instances[] = {
  &::dtproto::geometry_msgs::_Displacement_default_instance_._instance,
};

const char descriptor_table_protodef_dtProto_2fgeometry_5fmsgs_2fDisplacement_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n(dtProto/geometry_msgs/Displacement.pro"
  "to\022\025dtproto.geometry_msgs\"2\n\014Displacemen"
  "t\022\n\n\002a1\030\001 \001(\001\022\n\n\002a2\030\002 \001(\001\022\n\n\002a3\030\003 \001(\001b\006p"
  "roto3"
  ;
static ::_pbi::once_flag descriptor_table_dtProto_2fgeometry_5fmsgs_2fDisplacement_2eproto_once;
const ::_pbi::DescriptorTable descriptor_table_dtProto_2fgeometry_5fmsgs_2fDisplacement_2eproto = {
    false, false, 125, descriptor_table_protodef_dtProto_2fgeometry_5fmsgs_2fDisplacement_2eproto,
    "dtProto/geometry_msgs/Displacement.proto",
    &descriptor_table_dtProto_2fgeometry_5fmsgs_2fDisplacement_2eproto_once, nullptr, 0, 1,
    schemas, file_default_instances, TableStruct_dtProto_2fgeometry_5fmsgs_2fDisplacement_2eproto::offsets,
    file_level_metadata_dtProto_2fgeometry_5fmsgs_2fDisplacement_2eproto, file_level_enum_descriptors_dtProto_2fgeometry_5fmsgs_2fDisplacement_2eproto,
    file_level_service_descriptors_dtProto_2fgeometry_5fmsgs_2fDisplacement_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::_pbi::DescriptorTable* descriptor_table_dtProto_2fgeometry_5fmsgs_2fDisplacement_2eproto_getter() {
  return &descriptor_table_dtProto_2fgeometry_5fmsgs_2fDisplacement_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY2 static ::_pbi::AddDescriptorsRunner dynamic_init_dummy_dtProto_2fgeometry_5fmsgs_2fDisplacement_2eproto(&descriptor_table_dtProto_2fgeometry_5fmsgs_2fDisplacement_2eproto);
namespace dtproto {
namespace geometry_msgs {

// ===================================================================

class Displacement::_Internal {
 public:
};

Displacement::Displacement(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor(arena, is_message_owned);
  // @@protoc_insertion_point(arena_constructor:dtproto.geometry_msgs.Displacement)
}
Displacement::Displacement(const Displacement& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  Displacement* const _this = this; (void)_this;
  new (&_impl_) Impl_{
      decltype(_impl_.a1_){}
    , decltype(_impl_.a2_){}
    , decltype(_impl_.a3_){}
    , /*decltype(_impl_._cached_size_)*/{}};

  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::memcpy(&_impl_.a1_, &from._impl_.a1_,
    static_cast<size_t>(reinterpret_cast<char*>(&_impl_.a3_) -
    reinterpret_cast<char*>(&_impl_.a1_)) + sizeof(_impl_.a3_));
  // @@protoc_insertion_point(copy_constructor:dtproto.geometry_msgs.Displacement)
}

inline void Displacement::SharedCtor(
    ::_pb::Arena* arena, bool is_message_owned) {
  (void)arena;
  (void)is_message_owned;
  new (&_impl_) Impl_{
      decltype(_impl_.a1_){0}
    , decltype(_impl_.a2_){0}
    , decltype(_impl_.a3_){0}
    , /*decltype(_impl_._cached_size_)*/{}
  };
}

Displacement::~Displacement() {
  // @@protoc_insertion_point(destructor:dtproto.geometry_msgs.Displacement)
  if (auto *arena = _internal_metadata_.DeleteReturnArena<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>()) {
  (void)arena;
    return;
  }
  SharedDtor();
}

inline void Displacement::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
}

void Displacement::SetCachedSize(int size) const {
  _impl_._cached_size_.Set(size);
}

void Displacement::Clear() {
// @@protoc_insertion_point(message_clear_start:dtproto.geometry_msgs.Displacement)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ::memset(&_impl_.a1_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&_impl_.a3_) -
      reinterpret_cast<char*>(&_impl_.a1_)) + sizeof(_impl_.a3_));
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* Displacement::_InternalParse(const char* ptr, ::_pbi::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::_pbi::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // double a1 = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 9)) {
          _impl_.a1_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // double a2 = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 17)) {
          _impl_.a2_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // double a3 = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 25)) {
          _impl_.a3_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
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

uint8_t* Displacement::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:dtproto.geometry_msgs.Displacement)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // double a1 = 1;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_a1 = this->_internal_a1();
  uint64_t raw_a1;
  memcpy(&raw_a1, &tmp_a1, sizeof(tmp_a1));
  if (raw_a1 != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteDoubleToArray(1, this->_internal_a1(), target);
  }

  // double a2 = 2;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_a2 = this->_internal_a2();
  uint64_t raw_a2;
  memcpy(&raw_a2, &tmp_a2, sizeof(tmp_a2));
  if (raw_a2 != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteDoubleToArray(2, this->_internal_a2(), target);
  }

  // double a3 = 3;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_a3 = this->_internal_a3();
  uint64_t raw_a3;
  memcpy(&raw_a3, &tmp_a3, sizeof(tmp_a3));
  if (raw_a3 != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteDoubleToArray(3, this->_internal_a3(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:dtproto.geometry_msgs.Displacement)
  return target;
}

size_t Displacement::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:dtproto.geometry_msgs.Displacement)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // double a1 = 1;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_a1 = this->_internal_a1();
  uint64_t raw_a1;
  memcpy(&raw_a1, &tmp_a1, sizeof(tmp_a1));
  if (raw_a1 != 0) {
    total_size += 1 + 8;
  }

  // double a2 = 2;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_a2 = this->_internal_a2();
  uint64_t raw_a2;
  memcpy(&raw_a2, &tmp_a2, sizeof(tmp_a2));
  if (raw_a2 != 0) {
    total_size += 1 + 8;
  }

  // double a3 = 3;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_a3 = this->_internal_a3();
  uint64_t raw_a3;
  memcpy(&raw_a3, &tmp_a3, sizeof(tmp_a3));
  if (raw_a3 != 0) {
    total_size += 1 + 8;
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_impl_._cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData Displacement::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSourceCheck,
    Displacement::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*Displacement::GetClassData() const { return &_class_data_; }


void Displacement::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg) {
  auto* const _this = static_cast<Displacement*>(&to_msg);
  auto& from = static_cast<const Displacement&>(from_msg);
  // @@protoc_insertion_point(class_specific_merge_from_start:dtproto.geometry_msgs.Displacement)
  GOOGLE_DCHECK_NE(&from, _this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_a1 = from._internal_a1();
  uint64_t raw_a1;
  memcpy(&raw_a1, &tmp_a1, sizeof(tmp_a1));
  if (raw_a1 != 0) {
    _this->_internal_set_a1(from._internal_a1());
  }
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_a2 = from._internal_a2();
  uint64_t raw_a2;
  memcpy(&raw_a2, &tmp_a2, sizeof(tmp_a2));
  if (raw_a2 != 0) {
    _this->_internal_set_a2(from._internal_a2());
  }
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_a3 = from._internal_a3();
  uint64_t raw_a3;
  memcpy(&raw_a3, &tmp_a3, sizeof(tmp_a3));
  if (raw_a3 != 0) {
    _this->_internal_set_a3(from._internal_a3());
  }
  _this->_internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void Displacement::CopyFrom(const Displacement& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:dtproto.geometry_msgs.Displacement)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Displacement::IsInitialized() const {
  return true;
}

void Displacement::InternalSwap(Displacement* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(Displacement, _impl_.a3_)
      + sizeof(Displacement::_impl_.a3_)
      - PROTOBUF_FIELD_OFFSET(Displacement, _impl_.a1_)>(
          reinterpret_cast<char*>(&_impl_.a1_),
          reinterpret_cast<char*>(&other->_impl_.a1_));
}

::PROTOBUF_NAMESPACE_ID::Metadata Displacement::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_dtProto_2fgeometry_5fmsgs_2fDisplacement_2eproto_getter, &descriptor_table_dtProto_2fgeometry_5fmsgs_2fDisplacement_2eproto_once,
      file_level_metadata_dtProto_2fgeometry_5fmsgs_2fDisplacement_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace geometry_msgs
}  // namespace dtproto
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::dtproto::geometry_msgs::Displacement*
Arena::CreateMaybeMessage< ::dtproto::geometry_msgs::Displacement >(Arena* arena) {
  return Arena::CreateMessageInternal< ::dtproto::geometry_msgs::Displacement >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>