// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: dtProto/robot_msgs/ControlCmd.proto

#include "dtProto/robot_msgs/ControlCmd.pb.h"

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
namespace robot_msgs {
PROTOBUF_CONSTEXPR ControlCmd::ControlCmd(
    ::_pbi::ConstantInitialized): _impl_{
    /*decltype(_impl_.arg_)*/{&::_pbi::fixed_address_empty_string, ::_pbi::ConstantInitialized{}}
  , /*decltype(_impl_.cmd_mode_)*/0
  , /*decltype(_impl_._cached_size_)*/{}} {}
struct ControlCmdDefaultTypeInternal {
  PROTOBUF_CONSTEXPR ControlCmdDefaultTypeInternal()
      : _instance(::_pbi::ConstantInitialized{}) {}
  ~ControlCmdDefaultTypeInternal() {}
  union {
    ControlCmd _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 ControlCmdDefaultTypeInternal _ControlCmd_default_instance_;
PROTOBUF_CONSTEXPR ControlCmdTimeStamped::ControlCmdTimeStamped(
    ::_pbi::ConstantInitialized): _impl_{
    /*decltype(_impl_.header_)*/nullptr
  , /*decltype(_impl_.cmd_)*/nullptr
  , /*decltype(_impl_._cached_size_)*/{}} {}
struct ControlCmdTimeStampedDefaultTypeInternal {
  PROTOBUF_CONSTEXPR ControlCmdTimeStampedDefaultTypeInternal()
      : _instance(::_pbi::ConstantInitialized{}) {}
  ~ControlCmdTimeStampedDefaultTypeInternal() {}
  union {
    ControlCmdTimeStamped _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 ControlCmdTimeStampedDefaultTypeInternal _ControlCmdTimeStamped_default_instance_;
}  // namespace robot_msgs
}  // namespace dtproto
static ::_pb::Metadata file_level_metadata_dtProto_2frobot_5fmsgs_2fControlCmd_2eproto[2];
static constexpr ::_pb::EnumDescriptor const** file_level_enum_descriptors_dtProto_2frobot_5fmsgs_2fControlCmd_2eproto = nullptr;
static constexpr ::_pb::ServiceDescriptor const** file_level_service_descriptors_dtProto_2frobot_5fmsgs_2fControlCmd_2eproto = nullptr;

const uint32_t TableStruct_dtProto_2frobot_5fmsgs_2fControlCmd_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::dtproto::robot_msgs::ControlCmd, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::dtproto::robot_msgs::ControlCmd, _impl_.cmd_mode_),
  PROTOBUF_FIELD_OFFSET(::dtproto::robot_msgs::ControlCmd, _impl_.arg_),
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::dtproto::robot_msgs::ControlCmdTimeStamped, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::dtproto::robot_msgs::ControlCmdTimeStamped, _impl_.header_),
  PROTOBUF_FIELD_OFFSET(::dtproto::robot_msgs::ControlCmdTimeStamped, _impl_.cmd_),
};
static const ::_pbi::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::dtproto::robot_msgs::ControlCmd)},
  { 8, -1, -1, sizeof(::dtproto::robot_msgs::ControlCmdTimeStamped)},
};

static const ::_pb::Message* const file_default_instances[] = {
  &::dtproto::robot_msgs::_ControlCmd_default_instance_._instance,
  &::dtproto::robot_msgs::_ControlCmdTimeStamped_default_instance_._instance,
};

const char descriptor_table_protodef_dtProto_2frobot_5fmsgs_2fControlCmd_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n#dtProto/robot_msgs/ControlCmd.proto\022\022d"
  "tproto.robot_msgs\032\035dtProto/std_msgs/Head"
  "er.proto\032 dtProto/geometry_msgs/Pose.pro"
  "to\032!dtProto/geometry_msgs/Twist.proto\"+\n"
  "\nControlCmd\022\020\n\010cmd_mode\030\001 \001(\005\022\013\n\003arg\030\002 \001"
  "(\t\"n\n\025ControlCmdTimeStamped\022(\n\006header\030\001 "
  "\001(\0132\030.dtproto.std_msgs.Header\022+\n\003cmd\030\002 \001"
  "(\0132\036.dtproto.robot_msgs.ControlCmdb\006prot"
  "o3"
  ;
static const ::_pbi::DescriptorTable* const descriptor_table_dtProto_2frobot_5fmsgs_2fControlCmd_2eproto_deps[3] = {
  &::descriptor_table_dtProto_2fgeometry_5fmsgs_2fPose_2eproto,
  &::descriptor_table_dtProto_2fgeometry_5fmsgs_2fTwist_2eproto,
  &::descriptor_table_dtProto_2fstd_5fmsgs_2fHeader_2eproto,
};
static ::_pbi::once_flag descriptor_table_dtProto_2frobot_5fmsgs_2fControlCmd_2eproto_once;
const ::_pbi::DescriptorTable descriptor_table_dtProto_2frobot_5fmsgs_2fControlCmd_2eproto = {
    false, false, 322, descriptor_table_protodef_dtProto_2frobot_5fmsgs_2fControlCmd_2eproto,
    "dtProto/robot_msgs/ControlCmd.proto",
    &descriptor_table_dtProto_2frobot_5fmsgs_2fControlCmd_2eproto_once, descriptor_table_dtProto_2frobot_5fmsgs_2fControlCmd_2eproto_deps, 3, 2,
    schemas, file_default_instances, TableStruct_dtProto_2frobot_5fmsgs_2fControlCmd_2eproto::offsets,
    file_level_metadata_dtProto_2frobot_5fmsgs_2fControlCmd_2eproto, file_level_enum_descriptors_dtProto_2frobot_5fmsgs_2fControlCmd_2eproto,
    file_level_service_descriptors_dtProto_2frobot_5fmsgs_2fControlCmd_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::_pbi::DescriptorTable* descriptor_table_dtProto_2frobot_5fmsgs_2fControlCmd_2eproto_getter() {
  return &descriptor_table_dtProto_2frobot_5fmsgs_2fControlCmd_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY2 static ::_pbi::AddDescriptorsRunner dynamic_init_dummy_dtProto_2frobot_5fmsgs_2fControlCmd_2eproto(&descriptor_table_dtProto_2frobot_5fmsgs_2fControlCmd_2eproto);
namespace dtproto {
namespace robot_msgs {

// ===================================================================

class ControlCmd::_Internal {
 public:
};

ControlCmd::ControlCmd(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor(arena, is_message_owned);
  // @@protoc_insertion_point(arena_constructor:dtproto.robot_msgs.ControlCmd)
}
ControlCmd::ControlCmd(const ControlCmd& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  ControlCmd* const _this = this; (void)_this;
  new (&_impl_) Impl_{
      decltype(_impl_.arg_){}
    , decltype(_impl_.cmd_mode_){}
    , /*decltype(_impl_._cached_size_)*/{}};

  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  _impl_.arg_.InitDefault();
  #ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
    _impl_.arg_.Set("", GetArenaForAllocation());
  #endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (!from._internal_arg().empty()) {
    _this->_impl_.arg_.Set(from._internal_arg(), 
      _this->GetArenaForAllocation());
  }
  _this->_impl_.cmd_mode_ = from._impl_.cmd_mode_;
  // @@protoc_insertion_point(copy_constructor:dtproto.robot_msgs.ControlCmd)
}

inline void ControlCmd::SharedCtor(
    ::_pb::Arena* arena, bool is_message_owned) {
  (void)arena;
  (void)is_message_owned;
  new (&_impl_) Impl_{
      decltype(_impl_.arg_){}
    , decltype(_impl_.cmd_mode_){0}
    , /*decltype(_impl_._cached_size_)*/{}
  };
  _impl_.arg_.InitDefault();
  #ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
    _impl_.arg_.Set("", GetArenaForAllocation());
  #endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
}

ControlCmd::~ControlCmd() {
  // @@protoc_insertion_point(destructor:dtproto.robot_msgs.ControlCmd)
  if (auto *arena = _internal_metadata_.DeleteReturnArena<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>()) {
  (void)arena;
    return;
  }
  SharedDtor();
}

inline void ControlCmd::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  _impl_.arg_.Destroy();
}

void ControlCmd::SetCachedSize(int size) const {
  _impl_._cached_size_.Set(size);
}

void ControlCmd::Clear() {
// @@protoc_insertion_point(message_clear_start:dtproto.robot_msgs.ControlCmd)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  _impl_.arg_.ClearToEmpty();
  _impl_.cmd_mode_ = 0;
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* ControlCmd::_InternalParse(const char* ptr, ::_pbi::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::_pbi::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // int32 cmd_mode = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 8)) {
          _impl_.cmd_mode_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // string arg = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 18)) {
          auto str = _internal_mutable_arg();
          ptr = ::_pbi::InlineGreedyStringParser(str, ptr, ctx);
          CHK_(ptr);
          CHK_(::_pbi::VerifyUTF8(str, "dtproto.robot_msgs.ControlCmd.arg"));
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

uint8_t* ControlCmd::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:dtproto.robot_msgs.ControlCmd)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // int32 cmd_mode = 1;
  if (this->_internal_cmd_mode() != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteInt32ToArray(1, this->_internal_cmd_mode(), target);
  }

  // string arg = 2;
  if (!this->_internal_arg().empty()) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::VerifyUtf8String(
      this->_internal_arg().data(), static_cast<int>(this->_internal_arg().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::SERIALIZE,
      "dtproto.robot_msgs.ControlCmd.arg");
    target = stream->WriteStringMaybeAliased(
        2, this->_internal_arg(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:dtproto.robot_msgs.ControlCmd)
  return target;
}

size_t ControlCmd::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:dtproto.robot_msgs.ControlCmd)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // string arg = 2;
  if (!this->_internal_arg().empty()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
        this->_internal_arg());
  }

  // int32 cmd_mode = 1;
  if (this->_internal_cmd_mode() != 0) {
    total_size += ::_pbi::WireFormatLite::Int32SizePlusOne(this->_internal_cmd_mode());
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_impl_._cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData ControlCmd::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSourceCheck,
    ControlCmd::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*ControlCmd::GetClassData() const { return &_class_data_; }


void ControlCmd::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg) {
  auto* const _this = static_cast<ControlCmd*>(&to_msg);
  auto& from = static_cast<const ControlCmd&>(from_msg);
  // @@protoc_insertion_point(class_specific_merge_from_start:dtproto.robot_msgs.ControlCmd)
  GOOGLE_DCHECK_NE(&from, _this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  if (!from._internal_arg().empty()) {
    _this->_internal_set_arg(from._internal_arg());
  }
  if (from._internal_cmd_mode() != 0) {
    _this->_internal_set_cmd_mode(from._internal_cmd_mode());
  }
  _this->_internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void ControlCmd::CopyFrom(const ControlCmd& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:dtproto.robot_msgs.ControlCmd)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ControlCmd::IsInitialized() const {
  return true;
}

void ControlCmd::InternalSwap(ControlCmd* other) {
  using std::swap;
  auto* lhs_arena = GetArenaForAllocation();
  auto* rhs_arena = other->GetArenaForAllocation();
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::InternalSwap(
      &_impl_.arg_, lhs_arena,
      &other->_impl_.arg_, rhs_arena
  );
  swap(_impl_.cmd_mode_, other->_impl_.cmd_mode_);
}

::PROTOBUF_NAMESPACE_ID::Metadata ControlCmd::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_dtProto_2frobot_5fmsgs_2fControlCmd_2eproto_getter, &descriptor_table_dtProto_2frobot_5fmsgs_2fControlCmd_2eproto_once,
      file_level_metadata_dtProto_2frobot_5fmsgs_2fControlCmd_2eproto[0]);
}

// ===================================================================

class ControlCmdTimeStamped::_Internal {
 public:
  static const ::dtproto::std_msgs::Header& header(const ControlCmdTimeStamped* msg);
  static const ::dtproto::robot_msgs::ControlCmd& cmd(const ControlCmdTimeStamped* msg);
};

const ::dtproto::std_msgs::Header&
ControlCmdTimeStamped::_Internal::header(const ControlCmdTimeStamped* msg) {
  return *msg->_impl_.header_;
}
const ::dtproto::robot_msgs::ControlCmd&
ControlCmdTimeStamped::_Internal::cmd(const ControlCmdTimeStamped* msg) {
  return *msg->_impl_.cmd_;
}
void ControlCmdTimeStamped::clear_header() {
  if (GetArenaForAllocation() == nullptr && _impl_.header_ != nullptr) {
    delete _impl_.header_;
  }
  _impl_.header_ = nullptr;
}
ControlCmdTimeStamped::ControlCmdTimeStamped(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor(arena, is_message_owned);
  // @@protoc_insertion_point(arena_constructor:dtproto.robot_msgs.ControlCmdTimeStamped)
}
ControlCmdTimeStamped::ControlCmdTimeStamped(const ControlCmdTimeStamped& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  ControlCmdTimeStamped* const _this = this; (void)_this;
  new (&_impl_) Impl_{
      decltype(_impl_.header_){nullptr}
    , decltype(_impl_.cmd_){nullptr}
    , /*decltype(_impl_._cached_size_)*/{}};

  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_header()) {
    _this->_impl_.header_ = new ::dtproto::std_msgs::Header(*from._impl_.header_);
  }
  if (from._internal_has_cmd()) {
    _this->_impl_.cmd_ = new ::dtproto::robot_msgs::ControlCmd(*from._impl_.cmd_);
  }
  // @@protoc_insertion_point(copy_constructor:dtproto.robot_msgs.ControlCmdTimeStamped)
}

inline void ControlCmdTimeStamped::SharedCtor(
    ::_pb::Arena* arena, bool is_message_owned) {
  (void)arena;
  (void)is_message_owned;
  new (&_impl_) Impl_{
      decltype(_impl_.header_){nullptr}
    , decltype(_impl_.cmd_){nullptr}
    , /*decltype(_impl_._cached_size_)*/{}
  };
}

ControlCmdTimeStamped::~ControlCmdTimeStamped() {
  // @@protoc_insertion_point(destructor:dtproto.robot_msgs.ControlCmdTimeStamped)
  if (auto *arena = _internal_metadata_.DeleteReturnArena<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>()) {
  (void)arena;
    return;
  }
  SharedDtor();
}

inline void ControlCmdTimeStamped::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete _impl_.header_;
  if (this != internal_default_instance()) delete _impl_.cmd_;
}

void ControlCmdTimeStamped::SetCachedSize(int size) const {
  _impl_._cached_size_.Set(size);
}

void ControlCmdTimeStamped::Clear() {
// @@protoc_insertion_point(message_clear_start:dtproto.robot_msgs.ControlCmdTimeStamped)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  if (GetArenaForAllocation() == nullptr && _impl_.header_ != nullptr) {
    delete _impl_.header_;
  }
  _impl_.header_ = nullptr;
  if (GetArenaForAllocation() == nullptr && _impl_.cmd_ != nullptr) {
    delete _impl_.cmd_;
  }
  _impl_.cmd_ = nullptr;
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* ControlCmdTimeStamped::_InternalParse(const char* ptr, ::_pbi::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::_pbi::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // .dtproto.std_msgs.Header header = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_header(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // .dtproto.robot_msgs.ControlCmd cmd = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 18)) {
          ptr = ctx->ParseMessage(_internal_mutable_cmd(), ptr);
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

uint8_t* ControlCmdTimeStamped::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:dtproto.robot_msgs.ControlCmdTimeStamped)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // .dtproto.std_msgs.Header header = 1;
  if (this->_internal_has_header()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(1, _Internal::header(this),
        _Internal::header(this).GetCachedSize(), target, stream);
  }

  // .dtproto.robot_msgs.ControlCmd cmd = 2;
  if (this->_internal_has_cmd()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(2, _Internal::cmd(this),
        _Internal::cmd(this).GetCachedSize(), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:dtproto.robot_msgs.ControlCmdTimeStamped)
  return target;
}

size_t ControlCmdTimeStamped::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:dtproto.robot_msgs.ControlCmdTimeStamped)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // .dtproto.std_msgs.Header header = 1;
  if (this->_internal_has_header()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *_impl_.header_);
  }

  // .dtproto.robot_msgs.ControlCmd cmd = 2;
  if (this->_internal_has_cmd()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *_impl_.cmd_);
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_impl_._cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData ControlCmdTimeStamped::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSourceCheck,
    ControlCmdTimeStamped::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*ControlCmdTimeStamped::GetClassData() const { return &_class_data_; }


void ControlCmdTimeStamped::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg) {
  auto* const _this = static_cast<ControlCmdTimeStamped*>(&to_msg);
  auto& from = static_cast<const ControlCmdTimeStamped&>(from_msg);
  // @@protoc_insertion_point(class_specific_merge_from_start:dtproto.robot_msgs.ControlCmdTimeStamped)
  GOOGLE_DCHECK_NE(&from, _this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  if (from._internal_has_header()) {
    _this->_internal_mutable_header()->::dtproto::std_msgs::Header::MergeFrom(
        from._internal_header());
  }
  if (from._internal_has_cmd()) {
    _this->_internal_mutable_cmd()->::dtproto::robot_msgs::ControlCmd::MergeFrom(
        from._internal_cmd());
  }
  _this->_internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void ControlCmdTimeStamped::CopyFrom(const ControlCmdTimeStamped& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:dtproto.robot_msgs.ControlCmdTimeStamped)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ControlCmdTimeStamped::IsInitialized() const {
  return true;
}

void ControlCmdTimeStamped::InternalSwap(ControlCmdTimeStamped* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(ControlCmdTimeStamped, _impl_.cmd_)
      + sizeof(ControlCmdTimeStamped::_impl_.cmd_)
      - PROTOBUF_FIELD_OFFSET(ControlCmdTimeStamped, _impl_.header_)>(
          reinterpret_cast<char*>(&_impl_.header_),
          reinterpret_cast<char*>(&other->_impl_.header_));
}

::PROTOBUF_NAMESPACE_ID::Metadata ControlCmdTimeStamped::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_dtProto_2frobot_5fmsgs_2fControlCmd_2eproto_getter, &descriptor_table_dtProto_2frobot_5fmsgs_2fControlCmd_2eproto_once,
      file_level_metadata_dtProto_2frobot_5fmsgs_2fControlCmd_2eproto[1]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace robot_msgs
}  // namespace dtproto
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::dtproto::robot_msgs::ControlCmd*
Arena::CreateMaybeMessage< ::dtproto::robot_msgs::ControlCmd >(Arena* arena) {
  return Arena::CreateMessageInternal< ::dtproto::robot_msgs::ControlCmd >(arena);
}
template<> PROTOBUF_NOINLINE ::dtproto::robot_msgs::ControlCmdTimeStamped*
Arena::CreateMaybeMessage< ::dtproto::robot_msgs::ControlCmdTimeStamped >(Arena* arena) {
  return Arena::CreateMessageInternal< ::dtproto::robot_msgs::ControlCmdTimeStamped >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
