// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: dtProto/std_msgs/PackedArray.proto

#include "dtProto/std_msgs/PackedArray.pb.h"

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
namespace std_msgs {
PROTOBUF_CONSTEXPR PackedDouble::PackedDouble(
    ::_pbi::ConstantInitialized): _impl_{
    /*decltype(_impl_.data_)*/{}
  , /*decltype(_impl_._cached_size_)*/{}} {}
struct PackedDoubleDefaultTypeInternal {
  PROTOBUF_CONSTEXPR PackedDoubleDefaultTypeInternal()
      : _instance(::_pbi::ConstantInitialized{}) {}
  ~PackedDoubleDefaultTypeInternal() {}
  union {
    PackedDouble _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 PackedDoubleDefaultTypeInternal _PackedDouble_default_instance_;
}  // namespace std_msgs
}  // namespace dtproto
static ::_pb::Metadata file_level_metadata_dtProto_2fstd_5fmsgs_2fPackedArray_2eproto[1];
static constexpr ::_pb::EnumDescriptor const** file_level_enum_descriptors_dtProto_2fstd_5fmsgs_2fPackedArray_2eproto = nullptr;
static constexpr ::_pb::ServiceDescriptor const** file_level_service_descriptors_dtProto_2fstd_5fmsgs_2fPackedArray_2eproto = nullptr;

const uint32_t TableStruct_dtProto_2fstd_5fmsgs_2fPackedArray_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::dtproto::std_msgs::PackedDouble, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::dtproto::std_msgs::PackedDouble, _impl_.data_),
};
static const ::_pbi::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::dtproto::std_msgs::PackedDouble)},
};

static const ::_pb::Message* const file_default_instances[] = {
  &::dtproto::std_msgs::_PackedDouble_default_instance_._instance,
};

const char descriptor_table_protodef_dtProto_2fstd_5fmsgs_2fPackedArray_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\"dtProto/std_msgs/PackedArray.proto\022\020dt"
  "proto.std_msgs\" \n\014PackedDouble\022\020\n\004data\030\001"
  " \003(\001B\002\020\001b\006proto3"
  ;
static ::_pbi::once_flag descriptor_table_dtProto_2fstd_5fmsgs_2fPackedArray_2eproto_once;
const ::_pbi::DescriptorTable descriptor_table_dtProto_2fstd_5fmsgs_2fPackedArray_2eproto = {
    false, false, 96, descriptor_table_protodef_dtProto_2fstd_5fmsgs_2fPackedArray_2eproto,
    "dtProto/std_msgs/PackedArray.proto",
    &descriptor_table_dtProto_2fstd_5fmsgs_2fPackedArray_2eproto_once, nullptr, 0, 1,
    schemas, file_default_instances, TableStruct_dtProto_2fstd_5fmsgs_2fPackedArray_2eproto::offsets,
    file_level_metadata_dtProto_2fstd_5fmsgs_2fPackedArray_2eproto, file_level_enum_descriptors_dtProto_2fstd_5fmsgs_2fPackedArray_2eproto,
    file_level_service_descriptors_dtProto_2fstd_5fmsgs_2fPackedArray_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::_pbi::DescriptorTable* descriptor_table_dtProto_2fstd_5fmsgs_2fPackedArray_2eproto_getter() {
  return &descriptor_table_dtProto_2fstd_5fmsgs_2fPackedArray_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY2 static ::_pbi::AddDescriptorsRunner dynamic_init_dummy_dtProto_2fstd_5fmsgs_2fPackedArray_2eproto(&descriptor_table_dtProto_2fstd_5fmsgs_2fPackedArray_2eproto);
namespace dtproto {
namespace std_msgs {

// ===================================================================

class PackedDouble::_Internal {
 public:
};

PackedDouble::PackedDouble(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor(arena, is_message_owned);
  // @@protoc_insertion_point(arena_constructor:dtproto.std_msgs.PackedDouble)
}
PackedDouble::PackedDouble(const PackedDouble& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  PackedDouble* const _this = this; (void)_this;
  new (&_impl_) Impl_{
      decltype(_impl_.data_){from._impl_.data_}
    , /*decltype(_impl_._cached_size_)*/{}};

  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:dtproto.std_msgs.PackedDouble)
}

inline void PackedDouble::SharedCtor(
    ::_pb::Arena* arena, bool is_message_owned) {
  (void)arena;
  (void)is_message_owned;
  new (&_impl_) Impl_{
      decltype(_impl_.data_){arena}
    , /*decltype(_impl_._cached_size_)*/{}
  };
}

PackedDouble::~PackedDouble() {
  // @@protoc_insertion_point(destructor:dtproto.std_msgs.PackedDouble)
  if (auto *arena = _internal_metadata_.DeleteReturnArena<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>()) {
  (void)arena;
    return;
  }
  SharedDtor();
}

inline void PackedDouble::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  _impl_.data_.~RepeatedField();
}

void PackedDouble::SetCachedSize(int size) const {
  _impl_._cached_size_.Set(size);
}

void PackedDouble::Clear() {
// @@protoc_insertion_point(message_clear_start:dtproto.std_msgs.PackedDouble)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  _impl_.data_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* PackedDouble::_InternalParse(const char* ptr, ::_pbi::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::_pbi::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // repeated double data = 1 [packed = true];
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 10)) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::PackedDoubleParser(_internal_mutable_data(), ptr, ctx);
          CHK_(ptr);
        } else if (static_cast<uint8_t>(tag) == 9) {
          _internal_add_data(::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr));
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

uint8_t* PackedDouble::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:dtproto.std_msgs.PackedDouble)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated double data = 1 [packed = true];
  if (this->_internal_data_size() > 0) {
    target = stream->WriteFixedPacked(1, _internal_data(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:dtproto.std_msgs.PackedDouble)
  return target;
}

size_t PackedDouble::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:dtproto.std_msgs.PackedDouble)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated double data = 1 [packed = true];
  {
    unsigned int count = static_cast<unsigned int>(this->_internal_data_size());
    size_t data_size = 8UL * count;
    if (data_size > 0) {
      total_size += 1 +
        ::_pbi::WireFormatLite::Int32Size(static_cast<int32_t>(data_size));
    }
    total_size += data_size;
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_impl_._cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData PackedDouble::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSourceCheck,
    PackedDouble::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*PackedDouble::GetClassData() const { return &_class_data_; }


void PackedDouble::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg) {
  auto* const _this = static_cast<PackedDouble*>(&to_msg);
  auto& from = static_cast<const PackedDouble&>(from_msg);
  // @@protoc_insertion_point(class_specific_merge_from_start:dtproto.std_msgs.PackedDouble)
  GOOGLE_DCHECK_NE(&from, _this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  _this->_impl_.data_.MergeFrom(from._impl_.data_);
  _this->_internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void PackedDouble::CopyFrom(const PackedDouble& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:dtproto.std_msgs.PackedDouble)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool PackedDouble::IsInitialized() const {
  return true;
}

void PackedDouble::InternalSwap(PackedDouble* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  _impl_.data_.InternalSwap(&other->_impl_.data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata PackedDouble::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_dtProto_2fstd_5fmsgs_2fPackedArray_2eproto_getter, &descriptor_table_dtProto_2fstd_5fmsgs_2fPackedArray_2eproto_once,
      file_level_metadata_dtProto_2fstd_5fmsgs_2fPackedArray_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace std_msgs
}  // namespace dtproto
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::dtproto::std_msgs::PackedDouble*
Arena::CreateMaybeMessage< ::dtproto::std_msgs::PackedDouble >(Arena* arena) {
  return Arena::CreateMessageInternal< ::dtproto::std_msgs::PackedDouble >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>