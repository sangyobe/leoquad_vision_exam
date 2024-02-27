// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: dtProto/std_msgs/State.proto

#include "dtProto/std_msgs/State.pb.h"

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
PROTOBUF_CONSTEXPR State::State(
    ::_pbi::ConstantInitialized): _impl_{
    /*decltype(_impl_.state_)*/nullptr
  , /*decltype(_impl_._cached_size_)*/{}} {}
struct StateDefaultTypeInternal {
  PROTOBUF_CONSTEXPR StateDefaultTypeInternal()
      : _instance(::_pbi::ConstantInitialized{}) {}
  ~StateDefaultTypeInternal() {}
  union {
    State _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 StateDefaultTypeInternal _State_default_instance_;
}  // namespace std_msgs
}  // namespace dtproto
static ::_pb::Metadata file_level_metadata_dtProto_2fstd_5fmsgs_2fState_2eproto[1];
static constexpr ::_pb::EnumDescriptor const** file_level_enum_descriptors_dtProto_2fstd_5fmsgs_2fState_2eproto = nullptr;
static constexpr ::_pb::ServiceDescriptor const** file_level_service_descriptors_dtProto_2fstd_5fmsgs_2fState_2eproto = nullptr;

const uint32_t TableStruct_dtProto_2fstd_5fmsgs_2fState_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::dtproto::std_msgs::State, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::dtproto::std_msgs::State, _impl_.state_),
};
static const ::_pbi::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::dtproto::std_msgs::State)},
};

static const ::_pb::Message* const file_default_instances[] = {
  &::dtproto::std_msgs::_State_default_instance_._instance,
};

const char descriptor_table_protodef_dtProto_2fstd_5fmsgs_2fState_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\034dtProto/std_msgs/State.proto\022\020dtproto."
  "std_msgs\032\031google/protobuf/any.proto\",\n\005S"
  "tate\022#\n\005state\030\002 \001(\0132\024.google.protobuf.An"
  "yb\006proto3"
  ;
static const ::_pbi::DescriptorTable* const descriptor_table_dtProto_2fstd_5fmsgs_2fState_2eproto_deps[1] = {
  &::descriptor_table_google_2fprotobuf_2fany_2eproto,
};
static ::_pbi::once_flag descriptor_table_dtProto_2fstd_5fmsgs_2fState_2eproto_once;
const ::_pbi::DescriptorTable descriptor_table_dtProto_2fstd_5fmsgs_2fState_2eproto = {
    false, false, 129, descriptor_table_protodef_dtProto_2fstd_5fmsgs_2fState_2eproto,
    "dtProto/std_msgs/State.proto",
    &descriptor_table_dtProto_2fstd_5fmsgs_2fState_2eproto_once, descriptor_table_dtProto_2fstd_5fmsgs_2fState_2eproto_deps, 1, 1,
    schemas, file_default_instances, TableStruct_dtProto_2fstd_5fmsgs_2fState_2eproto::offsets,
    file_level_metadata_dtProto_2fstd_5fmsgs_2fState_2eproto, file_level_enum_descriptors_dtProto_2fstd_5fmsgs_2fState_2eproto,
    file_level_service_descriptors_dtProto_2fstd_5fmsgs_2fState_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::_pbi::DescriptorTable* descriptor_table_dtProto_2fstd_5fmsgs_2fState_2eproto_getter() {
  return &descriptor_table_dtProto_2fstd_5fmsgs_2fState_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY2 static ::_pbi::AddDescriptorsRunner dynamic_init_dummy_dtProto_2fstd_5fmsgs_2fState_2eproto(&descriptor_table_dtProto_2fstd_5fmsgs_2fState_2eproto);
namespace dtproto {
namespace std_msgs {

// ===================================================================

class State::_Internal {
 public:
  static const ::PROTOBUF_NAMESPACE_ID::Any& state(const State* msg);
};

const ::PROTOBUF_NAMESPACE_ID::Any&
State::_Internal::state(const State* msg) {
  return *msg->_impl_.state_;
}
void State::clear_state() {
  if (GetArenaForAllocation() == nullptr && _impl_.state_ != nullptr) {
    delete _impl_.state_;
  }
  _impl_.state_ = nullptr;
}
State::State(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor(arena, is_message_owned);
  // @@protoc_insertion_point(arena_constructor:dtproto.std_msgs.State)
}
State::State(const State& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  State* const _this = this; (void)_this;
  new (&_impl_) Impl_{
      decltype(_impl_.state_){nullptr}
    , /*decltype(_impl_._cached_size_)*/{}};

  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_state()) {
    _this->_impl_.state_ = new ::PROTOBUF_NAMESPACE_ID::Any(*from._impl_.state_);
  }
  // @@protoc_insertion_point(copy_constructor:dtproto.std_msgs.State)
}

inline void State::SharedCtor(
    ::_pb::Arena* arena, bool is_message_owned) {
  (void)arena;
  (void)is_message_owned;
  new (&_impl_) Impl_{
      decltype(_impl_.state_){nullptr}
    , /*decltype(_impl_._cached_size_)*/{}
  };
}

State::~State() {
  // @@protoc_insertion_point(destructor:dtproto.std_msgs.State)
  if (auto *arena = _internal_metadata_.DeleteReturnArena<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>()) {
  (void)arena;
    return;
  }
  SharedDtor();
}

inline void State::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete _impl_.state_;
}

void State::SetCachedSize(int size) const {
  _impl_._cached_size_.Set(size);
}

void State::Clear() {
// @@protoc_insertion_point(message_clear_start:dtproto.std_msgs.State)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  if (GetArenaForAllocation() == nullptr && _impl_.state_ != nullptr) {
    delete _impl_.state_;
  }
  _impl_.state_ = nullptr;
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* State::_InternalParse(const char* ptr, ::_pbi::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::_pbi::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // .google.protobuf.Any state = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 18)) {
          ptr = ctx->ParseMessage(_internal_mutable_state(), ptr);
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

uint8_t* State::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:dtproto.std_msgs.State)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // .google.protobuf.Any state = 2;
  if (this->_internal_has_state()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(2, _Internal::state(this),
        _Internal::state(this).GetCachedSize(), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:dtproto.std_msgs.State)
  return target;
}

size_t State::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:dtproto.std_msgs.State)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // .google.protobuf.Any state = 2;
  if (this->_internal_has_state()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *_impl_.state_);
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_impl_._cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData State::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSourceCheck,
    State::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*State::GetClassData() const { return &_class_data_; }


void State::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg) {
  auto* const _this = static_cast<State*>(&to_msg);
  auto& from = static_cast<const State&>(from_msg);
  // @@protoc_insertion_point(class_specific_merge_from_start:dtproto.std_msgs.State)
  GOOGLE_DCHECK_NE(&from, _this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  if (from._internal_has_state()) {
    _this->_internal_mutable_state()->::PROTOBUF_NAMESPACE_ID::Any::MergeFrom(
        from._internal_state());
  }
  _this->_internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void State::CopyFrom(const State& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:dtproto.std_msgs.State)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool State::IsInitialized() const {
  return true;
}

void State::InternalSwap(State* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_impl_.state_, other->_impl_.state_);
}

::PROTOBUF_NAMESPACE_ID::Metadata State::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_dtProto_2fstd_5fmsgs_2fState_2eproto_getter, &descriptor_table_dtProto_2fstd_5fmsgs_2fState_2eproto_once,
      file_level_metadata_dtProto_2fstd_5fmsgs_2fState_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace std_msgs
}  // namespace dtproto
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::dtproto::std_msgs::State*
Arena::CreateMaybeMessage< ::dtproto::std_msgs::State >(Arena* arena) {
  return Arena::CreateMessageInternal< ::dtproto::std_msgs::State >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
