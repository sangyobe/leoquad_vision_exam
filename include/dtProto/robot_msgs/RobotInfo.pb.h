// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: dtProto/robot_msgs/RobotInfo.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_dtProto_2frobot_5fmsgs_2fRobotInfo_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_dtProto_2frobot_5fmsgs_2fRobotInfo_2eproto

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
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_dtProto_2frobot_5fmsgs_2fRobotInfo_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_dtProto_2frobot_5fmsgs_2fRobotInfo_2eproto {
  static const uint32_t offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_dtProto_2frobot_5fmsgs_2fRobotInfo_2eproto;
namespace dtproto {
namespace robot_msgs {
class RobotInfo;
struct RobotInfoDefaultTypeInternal;
extern RobotInfoDefaultTypeInternal _RobotInfo_default_instance_;
}  // namespace robot_msgs
}  // namespace dtproto
PROTOBUF_NAMESPACE_OPEN
template<> ::dtproto::robot_msgs::RobotInfo* Arena::CreateMaybeMessage<::dtproto::robot_msgs::RobotInfo>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace dtproto {
namespace robot_msgs {

// ===================================================================

class RobotInfo final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:dtproto.robot_msgs.RobotInfo) */ {
 public:
  inline RobotInfo() : RobotInfo(nullptr) {}
  ~RobotInfo() override;
  explicit PROTOBUF_CONSTEXPR RobotInfo(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  RobotInfo(const RobotInfo& from);
  RobotInfo(RobotInfo&& from) noexcept
    : RobotInfo() {
    *this = ::std::move(from);
  }

  inline RobotInfo& operator=(const RobotInfo& from) {
    CopyFrom(from);
    return *this;
  }
  inline RobotInfo& operator=(RobotInfo&& from) noexcept {
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
  static const RobotInfo& default_instance() {
    return *internal_default_instance();
  }
  static inline const RobotInfo* internal_default_instance() {
    return reinterpret_cast<const RobotInfo*>(
               &_RobotInfo_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(RobotInfo& a, RobotInfo& b) {
    a.Swap(&b);
  }
  inline void Swap(RobotInfo* other) {
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
  void UnsafeArenaSwap(RobotInfo* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  RobotInfo* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<RobotInfo>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const RobotInfo& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom( const RobotInfo& from) {
    RobotInfo::MergeImpl(*this, from);
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
  void InternalSwap(RobotInfo* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "dtproto.robot_msgs.RobotInfo";
  }
  protected:
  explicit RobotInfo(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kNameFieldNumber = 1,
    kAuthorFieldNumber = 2,
    kVersionFieldNumber = 3,
    kDescriptionFieldNumber = 4,
    kSerialFieldNumber = 5,
    kTypeFieldNumber = 6,
    kIdFieldNumber = 7,
    kDofFieldNumber = 8,
  };
  // string name = 1;
  void clear_name();
  const std::string& name() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_name(ArgT0&& arg0, ArgT... args);
  std::string* mutable_name();
  PROTOBUF_NODISCARD std::string* release_name();
  void set_allocated_name(std::string* name);
  private:
  const std::string& _internal_name() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_name(const std::string& value);
  std::string* _internal_mutable_name();
  public:

  // string author = 2;
  void clear_author();
  const std::string& author() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_author(ArgT0&& arg0, ArgT... args);
  std::string* mutable_author();
  PROTOBUF_NODISCARD std::string* release_author();
  void set_allocated_author(std::string* author);
  private:
  const std::string& _internal_author() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_author(const std::string& value);
  std::string* _internal_mutable_author();
  public:

  // string version = 3;
  void clear_version();
  const std::string& version() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_version(ArgT0&& arg0, ArgT... args);
  std::string* mutable_version();
  PROTOBUF_NODISCARD std::string* release_version();
  void set_allocated_version(std::string* version);
  private:
  const std::string& _internal_version() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_version(const std::string& value);
  std::string* _internal_mutable_version();
  public:

  // string description = 4;
  void clear_description();
  const std::string& description() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_description(ArgT0&& arg0, ArgT... args);
  std::string* mutable_description();
  PROTOBUF_NODISCARD std::string* release_description();
  void set_allocated_description(std::string* description);
  private:
  const std::string& _internal_description() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_description(const std::string& value);
  std::string* _internal_mutable_description();
  public:

  // string serial = 5;
  void clear_serial();
  const std::string& serial() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_serial(ArgT0&& arg0, ArgT... args);
  std::string* mutable_serial();
  PROTOBUF_NODISCARD std::string* release_serial();
  void set_allocated_serial(std::string* serial);
  private:
  const std::string& _internal_serial() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_serial(const std::string& value);
  std::string* _internal_mutable_serial();
  public:

  // uint32 type = 6;
  void clear_type();
  uint32_t type() const;
  void set_type(uint32_t value);
  private:
  uint32_t _internal_type() const;
  void _internal_set_type(uint32_t value);
  public:

  // uint32 id = 7;
  void clear_id();
  uint32_t id() const;
  void set_id(uint32_t value);
  private:
  uint32_t _internal_id() const;
  void _internal_set_id(uint32_t value);
  public:

  // uint32 dof = 8;
  void clear_dof();
  uint32_t dof() const;
  void set_dof(uint32_t value);
  private:
  uint32_t _internal_dof() const;
  void _internal_set_dof(uint32_t value);
  public:

  // @@protoc_insertion_point(class_scope:dtproto.robot_msgs.RobotInfo)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  struct Impl_ {
    ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr name_;
    ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr author_;
    ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr version_;
    ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr description_;
    ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr serial_;
    uint32_t type_;
    uint32_t id_;
    uint32_t dof_;
    mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  };
  union { Impl_ _impl_; };
  friend struct ::TableStruct_dtProto_2frobot_5fmsgs_2fRobotInfo_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// RobotInfo

// string name = 1;
inline void RobotInfo::clear_name() {
  _impl_.name_.ClearToEmpty();
}
inline const std::string& RobotInfo::name() const {
  // @@protoc_insertion_point(field_get:dtproto.robot_msgs.RobotInfo.name)
  return _internal_name();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void RobotInfo::set_name(ArgT0&& arg0, ArgT... args) {
 
 _impl_.name_.Set(static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:dtproto.robot_msgs.RobotInfo.name)
}
inline std::string* RobotInfo::mutable_name() {
  std::string* _s = _internal_mutable_name();
  // @@protoc_insertion_point(field_mutable:dtproto.robot_msgs.RobotInfo.name)
  return _s;
}
inline const std::string& RobotInfo::_internal_name() const {
  return _impl_.name_.Get();
}
inline void RobotInfo::_internal_set_name(const std::string& value) {
  
  _impl_.name_.Set(value, GetArenaForAllocation());
}
inline std::string* RobotInfo::_internal_mutable_name() {
  
  return _impl_.name_.Mutable(GetArenaForAllocation());
}
inline std::string* RobotInfo::release_name() {
  // @@protoc_insertion_point(field_release:dtproto.robot_msgs.RobotInfo.name)
  return _impl_.name_.Release();
}
inline void RobotInfo::set_allocated_name(std::string* name) {
  if (name != nullptr) {
    
  } else {
    
  }
  _impl_.name_.SetAllocated(name, GetArenaForAllocation());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (_impl_.name_.IsDefault()) {
    _impl_.name_.Set("", GetArenaForAllocation());
  }
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  // @@protoc_insertion_point(field_set_allocated:dtproto.robot_msgs.RobotInfo.name)
}

// string author = 2;
inline void RobotInfo::clear_author() {
  _impl_.author_.ClearToEmpty();
}
inline const std::string& RobotInfo::author() const {
  // @@protoc_insertion_point(field_get:dtproto.robot_msgs.RobotInfo.author)
  return _internal_author();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void RobotInfo::set_author(ArgT0&& arg0, ArgT... args) {
 
 _impl_.author_.Set(static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:dtproto.robot_msgs.RobotInfo.author)
}
inline std::string* RobotInfo::mutable_author() {
  std::string* _s = _internal_mutable_author();
  // @@protoc_insertion_point(field_mutable:dtproto.robot_msgs.RobotInfo.author)
  return _s;
}
inline const std::string& RobotInfo::_internal_author() const {
  return _impl_.author_.Get();
}
inline void RobotInfo::_internal_set_author(const std::string& value) {
  
  _impl_.author_.Set(value, GetArenaForAllocation());
}
inline std::string* RobotInfo::_internal_mutable_author() {
  
  return _impl_.author_.Mutable(GetArenaForAllocation());
}
inline std::string* RobotInfo::release_author() {
  // @@protoc_insertion_point(field_release:dtproto.robot_msgs.RobotInfo.author)
  return _impl_.author_.Release();
}
inline void RobotInfo::set_allocated_author(std::string* author) {
  if (author != nullptr) {
    
  } else {
    
  }
  _impl_.author_.SetAllocated(author, GetArenaForAllocation());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (_impl_.author_.IsDefault()) {
    _impl_.author_.Set("", GetArenaForAllocation());
  }
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  // @@protoc_insertion_point(field_set_allocated:dtproto.robot_msgs.RobotInfo.author)
}

// string version = 3;
inline void RobotInfo::clear_version() {
  _impl_.version_.ClearToEmpty();
}
inline const std::string& RobotInfo::version() const {
  // @@protoc_insertion_point(field_get:dtproto.robot_msgs.RobotInfo.version)
  return _internal_version();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void RobotInfo::set_version(ArgT0&& arg0, ArgT... args) {
 
 _impl_.version_.Set(static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:dtproto.robot_msgs.RobotInfo.version)
}
inline std::string* RobotInfo::mutable_version() {
  std::string* _s = _internal_mutable_version();
  // @@protoc_insertion_point(field_mutable:dtproto.robot_msgs.RobotInfo.version)
  return _s;
}
inline const std::string& RobotInfo::_internal_version() const {
  return _impl_.version_.Get();
}
inline void RobotInfo::_internal_set_version(const std::string& value) {
  
  _impl_.version_.Set(value, GetArenaForAllocation());
}
inline std::string* RobotInfo::_internal_mutable_version() {
  
  return _impl_.version_.Mutable(GetArenaForAllocation());
}
inline std::string* RobotInfo::release_version() {
  // @@protoc_insertion_point(field_release:dtproto.robot_msgs.RobotInfo.version)
  return _impl_.version_.Release();
}
inline void RobotInfo::set_allocated_version(std::string* version) {
  if (version != nullptr) {
    
  } else {
    
  }
  _impl_.version_.SetAllocated(version, GetArenaForAllocation());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (_impl_.version_.IsDefault()) {
    _impl_.version_.Set("", GetArenaForAllocation());
  }
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  // @@protoc_insertion_point(field_set_allocated:dtproto.robot_msgs.RobotInfo.version)
}

// string description = 4;
inline void RobotInfo::clear_description() {
  _impl_.description_.ClearToEmpty();
}
inline const std::string& RobotInfo::description() const {
  // @@protoc_insertion_point(field_get:dtproto.robot_msgs.RobotInfo.description)
  return _internal_description();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void RobotInfo::set_description(ArgT0&& arg0, ArgT... args) {
 
 _impl_.description_.Set(static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:dtproto.robot_msgs.RobotInfo.description)
}
inline std::string* RobotInfo::mutable_description() {
  std::string* _s = _internal_mutable_description();
  // @@protoc_insertion_point(field_mutable:dtproto.robot_msgs.RobotInfo.description)
  return _s;
}
inline const std::string& RobotInfo::_internal_description() const {
  return _impl_.description_.Get();
}
inline void RobotInfo::_internal_set_description(const std::string& value) {
  
  _impl_.description_.Set(value, GetArenaForAllocation());
}
inline std::string* RobotInfo::_internal_mutable_description() {
  
  return _impl_.description_.Mutable(GetArenaForAllocation());
}
inline std::string* RobotInfo::release_description() {
  // @@protoc_insertion_point(field_release:dtproto.robot_msgs.RobotInfo.description)
  return _impl_.description_.Release();
}
inline void RobotInfo::set_allocated_description(std::string* description) {
  if (description != nullptr) {
    
  } else {
    
  }
  _impl_.description_.SetAllocated(description, GetArenaForAllocation());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (_impl_.description_.IsDefault()) {
    _impl_.description_.Set("", GetArenaForAllocation());
  }
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  // @@protoc_insertion_point(field_set_allocated:dtproto.robot_msgs.RobotInfo.description)
}

// string serial = 5;
inline void RobotInfo::clear_serial() {
  _impl_.serial_.ClearToEmpty();
}
inline const std::string& RobotInfo::serial() const {
  // @@protoc_insertion_point(field_get:dtproto.robot_msgs.RobotInfo.serial)
  return _internal_serial();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void RobotInfo::set_serial(ArgT0&& arg0, ArgT... args) {
 
 _impl_.serial_.Set(static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:dtproto.robot_msgs.RobotInfo.serial)
}
inline std::string* RobotInfo::mutable_serial() {
  std::string* _s = _internal_mutable_serial();
  // @@protoc_insertion_point(field_mutable:dtproto.robot_msgs.RobotInfo.serial)
  return _s;
}
inline const std::string& RobotInfo::_internal_serial() const {
  return _impl_.serial_.Get();
}
inline void RobotInfo::_internal_set_serial(const std::string& value) {
  
  _impl_.serial_.Set(value, GetArenaForAllocation());
}
inline std::string* RobotInfo::_internal_mutable_serial() {
  
  return _impl_.serial_.Mutable(GetArenaForAllocation());
}
inline std::string* RobotInfo::release_serial() {
  // @@protoc_insertion_point(field_release:dtproto.robot_msgs.RobotInfo.serial)
  return _impl_.serial_.Release();
}
inline void RobotInfo::set_allocated_serial(std::string* serial) {
  if (serial != nullptr) {
    
  } else {
    
  }
  _impl_.serial_.SetAllocated(serial, GetArenaForAllocation());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (_impl_.serial_.IsDefault()) {
    _impl_.serial_.Set("", GetArenaForAllocation());
  }
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  // @@protoc_insertion_point(field_set_allocated:dtproto.robot_msgs.RobotInfo.serial)
}

// uint32 type = 6;
inline void RobotInfo::clear_type() {
  _impl_.type_ = 0u;
}
inline uint32_t RobotInfo::_internal_type() const {
  return _impl_.type_;
}
inline uint32_t RobotInfo::type() const {
  // @@protoc_insertion_point(field_get:dtproto.robot_msgs.RobotInfo.type)
  return _internal_type();
}
inline void RobotInfo::_internal_set_type(uint32_t value) {
  
  _impl_.type_ = value;
}
inline void RobotInfo::set_type(uint32_t value) {
  _internal_set_type(value);
  // @@protoc_insertion_point(field_set:dtproto.robot_msgs.RobotInfo.type)
}

// uint32 id = 7;
inline void RobotInfo::clear_id() {
  _impl_.id_ = 0u;
}
inline uint32_t RobotInfo::_internal_id() const {
  return _impl_.id_;
}
inline uint32_t RobotInfo::id() const {
  // @@protoc_insertion_point(field_get:dtproto.robot_msgs.RobotInfo.id)
  return _internal_id();
}
inline void RobotInfo::_internal_set_id(uint32_t value) {
  
  _impl_.id_ = value;
}
inline void RobotInfo::set_id(uint32_t value) {
  _internal_set_id(value);
  // @@protoc_insertion_point(field_set:dtproto.robot_msgs.RobotInfo.id)
}

// uint32 dof = 8;
inline void RobotInfo::clear_dof() {
  _impl_.dof_ = 0u;
}
inline uint32_t RobotInfo::_internal_dof() const {
  return _impl_.dof_;
}
inline uint32_t RobotInfo::dof() const {
  // @@protoc_insertion_point(field_get:dtproto.robot_msgs.RobotInfo.dof)
  return _internal_dof();
}
inline void RobotInfo::_internal_set_dof(uint32_t value) {
  
  _impl_.dof_ = value;
}
inline void RobotInfo::set_dof(uint32_t value) {
  _internal_set_dof(value);
  // @@protoc_insertion_point(field_set:dtproto.robot_msgs.RobotInfo.dof)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace robot_msgs
}  // namespace dtproto

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_dtProto_2frobot_5fmsgs_2fRobotInfo_2eproto
