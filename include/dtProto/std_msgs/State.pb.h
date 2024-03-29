// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: dtProto/std_msgs/State.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_dtProto_2fstd_5fmsgs_2fState_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_dtProto_2fstd_5fmsgs_2fState_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3021000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3021012 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
#include <google/protobuf/any.pb.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_dtProto_2fstd_5fmsgs_2fState_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_dtProto_2fstd_5fmsgs_2fState_2eproto {
  static const uint32_t offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_dtProto_2fstd_5fmsgs_2fState_2eproto;
namespace dtproto {
namespace std_msgs {
class State;
struct StateDefaultTypeInternal;
extern StateDefaultTypeInternal _State_default_instance_;
}  // namespace std_msgs
}  // namespace dtproto
PROTOBUF_NAMESPACE_OPEN
template<> ::dtproto::std_msgs::State* Arena::CreateMaybeMessage<::dtproto::std_msgs::State>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace dtproto {
namespace std_msgs {

// ===================================================================

class State final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:dtproto.std_msgs.State) */ {
 public:
  inline State() : State(nullptr) {}
  ~State() override;
  explicit PROTOBUF_CONSTEXPR State(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  State(const State& from);
  State(State&& from) noexcept
    : State() {
    *this = ::std::move(from);
  }

  inline State& operator=(const State& from) {
    CopyFrom(from);
    return *this;
  }
  inline State& operator=(State&& from) noexcept {
    if (this == &from) return *this;
    if (GetOwningArena() == from.GetOwningArena()
  #ifdef PROTOBUF_FORCE_COPY_IN_MOVE
        && GetOwningArena() != nullptr
  #endif  // !PROTOBUF_FORCE_COPY_IN_MOVE
    ) {
      InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return default_instance().GetMetadata().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return default_instance().GetMetadata().reflection;
  }
  static const State& default_instance() {
    return *internal_default_instance();
  }
  static inline const State* internal_default_instance() {
    return reinterpret_cast<const State*>(
               &_State_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(State& a, State& b) {
    a.Swap(&b);
  }
  inline void Swap(State* other) {
    if (other == this) return;
  #ifdef PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() != nullptr &&
        GetOwningArena() == other->GetOwningArena()) {
   #else  // PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() == other->GetOwningArena()) {
  #endif  // !PROTOBUF_FORCE_COPY_IN_SWAP
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(State* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  State* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<State>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const State& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom( const State& from) {
    State::MergeImpl(*this, from);
  }
  private:
  static void MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg);
  public:
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  uint8_t* _InternalSerialize(
      uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _impl_._cached_size_.Get(); }

  private:
  void SharedCtor(::PROTOBUF_NAMESPACE_ID::Arena* arena, bool is_message_owned);
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(State* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "dtproto.std_msgs.State";
  }
  protected:
  explicit State(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kStateFieldNumber = 2,
  };
  // .google.protobuf.Any state = 2;
  bool has_state() const;
  private:
  bool _internal_has_state() const;
  public:
  void clear_state();
  const ::PROTOBUF_NAMESPACE_ID::Any& state() const;
  PROTOBUF_NODISCARD ::PROTOBUF_NAMESPACE_ID::Any* release_state();
  ::PROTOBUF_NAMESPACE_ID::Any* mutable_state();
  void set_allocated_state(::PROTOBUF_NAMESPACE_ID::Any* state);
  private:
  const ::PROTOBUF_NAMESPACE_ID::Any& _internal_state() const;
  ::PROTOBUF_NAMESPACE_ID::Any* _internal_mutable_state();
  public:
  void unsafe_arena_set_allocated_state(
      ::PROTOBUF_NAMESPACE_ID::Any* state);
  ::PROTOBUF_NAMESPACE_ID::Any* unsafe_arena_release_state();

  // @@protoc_insertion_point(class_scope:dtproto.std_msgs.State)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  struct Impl_ {
    ::PROTOBUF_NAMESPACE_ID::Any* state_;
    mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  };
  union { Impl_ _impl_; };
  friend struct ::TableStruct_dtProto_2fstd_5fmsgs_2fState_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// State

// .google.protobuf.Any state = 2;
inline bool State::_internal_has_state() const {
  return this != internal_default_instance() && _impl_.state_ != nullptr;
}
inline bool State::has_state() const {
  return _internal_has_state();
}
inline const ::PROTOBUF_NAMESPACE_ID::Any& State::_internal_state() const {
  const ::PROTOBUF_NAMESPACE_ID::Any* p = _impl_.state_;
  return p != nullptr ? *p : reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Any&>(
      ::PROTOBUF_NAMESPACE_ID::_Any_default_instance_);
}
inline const ::PROTOBUF_NAMESPACE_ID::Any& State::state() const {
  // @@protoc_insertion_point(field_get:dtproto.std_msgs.State.state)
  return _internal_state();
}
inline void State::unsafe_arena_set_allocated_state(
    ::PROTOBUF_NAMESPACE_ID::Any* state) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.state_);
  }
  _impl_.state_ = state;
  if (state) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:dtproto.std_msgs.State.state)
}
inline ::PROTOBUF_NAMESPACE_ID::Any* State::release_state() {
  
  ::PROTOBUF_NAMESPACE_ID::Any* temp = _impl_.state_;
  _impl_.state_ = nullptr;
#ifdef PROTOBUF_FORCE_COPY_IN_RELEASE
  auto* old =  reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(temp);
  temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  if (GetArenaForAllocation() == nullptr) { delete old; }
#else  // PROTOBUF_FORCE_COPY_IN_RELEASE
  if (GetArenaForAllocation() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
#endif  // !PROTOBUF_FORCE_COPY_IN_RELEASE
  return temp;
}
inline ::PROTOBUF_NAMESPACE_ID::Any* State::unsafe_arena_release_state() {
  // @@protoc_insertion_point(field_release:dtproto.std_msgs.State.state)
  
  ::PROTOBUF_NAMESPACE_ID::Any* temp = _impl_.state_;
  _impl_.state_ = nullptr;
  return temp;
}
inline ::PROTOBUF_NAMESPACE_ID::Any* State::_internal_mutable_state() {
  
  if (_impl_.state_ == nullptr) {
    auto* p = CreateMaybeMessage<::PROTOBUF_NAMESPACE_ID::Any>(GetArenaForAllocation());
    _impl_.state_ = p;
  }
  return _impl_.state_;
}
inline ::PROTOBUF_NAMESPACE_ID::Any* State::mutable_state() {
  ::PROTOBUF_NAMESPACE_ID::Any* _msg = _internal_mutable_state();
  // @@protoc_insertion_point(field_mutable:dtproto.std_msgs.State.state)
  return _msg;
}
inline void State::set_allocated_state(::PROTOBUF_NAMESPACE_ID::Any* state) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.state_);
  }
  if (state) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalGetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(state));
    if (message_arena != submessage_arena) {
      state = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, state, submessage_arena);
    }
    
  } else {
    
  }
  _impl_.state_ = state;
  // @@protoc_insertion_point(field_set_allocated:dtproto.std_msgs.State.state)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace std_msgs
}  // namespace dtproto

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_dtProto_2fstd_5fmsgs_2fState_2eproto
