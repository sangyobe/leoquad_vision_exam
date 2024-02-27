// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: dtProto/robot_msgs/MoveControl.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_dtProto_2frobot_5fmsgs_2fMoveControl_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_dtProto_2frobot_5fmsgs_2fMoveControl_2eproto

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
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
#include "dtProto/std_msgs/Header.pb.h"
#include "dtProto/geometry_msgs/Pose.pb.h"
#include "dtProto/geometry_msgs/Twist.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_dtProto_2frobot_5fmsgs_2fMoveControl_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_dtProto_2frobot_5fmsgs_2fMoveControl_2eproto {
  static const uint32_t offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_dtProto_2frobot_5fmsgs_2fMoveControl_2eproto;
namespace dtproto {
namespace robot_msgs {
class MoveControl;
struct MoveControlDefaultTypeInternal;
extern MoveControlDefaultTypeInternal _MoveControl_default_instance_;
class MoveControlTimeStamped;
struct MoveControlTimeStampedDefaultTypeInternal;
extern MoveControlTimeStampedDefaultTypeInternal _MoveControlTimeStamped_default_instance_;
}  // namespace robot_msgs
}  // namespace dtproto
PROTOBUF_NAMESPACE_OPEN
template<> ::dtproto::robot_msgs::MoveControl* Arena::CreateMaybeMessage<::dtproto::robot_msgs::MoveControl>(Arena*);
template<> ::dtproto::robot_msgs::MoveControlTimeStamped* Arena::CreateMaybeMessage<::dtproto::robot_msgs::MoveControlTimeStamped>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace dtproto {
namespace robot_msgs {

enum MoveControl_ControlMode : int {
  MoveControl_ControlMode_POSE_ABSOLUTE = 0,
  MoveControl_ControlMode_POSE_RELATIVE = 1,
  MoveControl_ControlMode_VEL_ABSOLUBE = 2,
  MoveControl_ControlMode_VEL_RELATIVE = 3,
  MoveControl_ControlMode_MoveControl_ControlMode_INT_MIN_SENTINEL_DO_NOT_USE_ = std::numeric_limits<int32_t>::min(),
  MoveControl_ControlMode_MoveControl_ControlMode_INT_MAX_SENTINEL_DO_NOT_USE_ = std::numeric_limits<int32_t>::max()
};
bool MoveControl_ControlMode_IsValid(int value);
constexpr MoveControl_ControlMode MoveControl_ControlMode_ControlMode_MIN = MoveControl_ControlMode_POSE_ABSOLUTE;
constexpr MoveControl_ControlMode MoveControl_ControlMode_ControlMode_MAX = MoveControl_ControlMode_VEL_RELATIVE;
constexpr int MoveControl_ControlMode_ControlMode_ARRAYSIZE = MoveControl_ControlMode_ControlMode_MAX + 1;

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* MoveControl_ControlMode_descriptor();
template<typename T>
inline const std::string& MoveControl_ControlMode_Name(T enum_t_value) {
  static_assert(::std::is_same<T, MoveControl_ControlMode>::value ||
    ::std::is_integral<T>::value,
    "Incorrect type passed to function MoveControl_ControlMode_Name.");
  return ::PROTOBUF_NAMESPACE_ID::internal::NameOfEnum(
    MoveControl_ControlMode_descriptor(), enum_t_value);
}
inline bool MoveControl_ControlMode_Parse(
    ::PROTOBUF_NAMESPACE_ID::ConstStringParam name, MoveControl_ControlMode* value) {
  return ::PROTOBUF_NAMESPACE_ID::internal::ParseNamedEnum<MoveControl_ControlMode>(
    MoveControl_ControlMode_descriptor(), name, value);
}
// ===================================================================

class MoveControl final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:dtproto.robot_msgs.MoveControl) */ {
 public:
  inline MoveControl() : MoveControl(nullptr) {}
  ~MoveControl() override;
  explicit PROTOBUF_CONSTEXPR MoveControl(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  MoveControl(const MoveControl& from);
  MoveControl(MoveControl&& from) noexcept
    : MoveControl() {
    *this = ::std::move(from);
  }

  inline MoveControl& operator=(const MoveControl& from) {
    CopyFrom(from);
    return *this;
  }
  inline MoveControl& operator=(MoveControl&& from) noexcept {
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
  static const MoveControl& default_instance() {
    return *internal_default_instance();
  }
  static inline const MoveControl* internal_default_instance() {
    return reinterpret_cast<const MoveControl*>(
               &_MoveControl_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(MoveControl& a, MoveControl& b) {
    a.Swap(&b);
  }
  inline void Swap(MoveControl* other) {
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
  void UnsafeArenaSwap(MoveControl* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  MoveControl* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<MoveControl>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const MoveControl& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom( const MoveControl& from) {
    MoveControl::MergeImpl(*this, from);
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
  void InternalSwap(MoveControl* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "dtproto.robot_msgs.MoveControl";
  }
  protected:
  explicit MoveControl(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  typedef MoveControl_ControlMode ControlMode;
  static constexpr ControlMode POSE_ABSOLUTE =
    MoveControl_ControlMode_POSE_ABSOLUTE;
  static constexpr ControlMode POSE_RELATIVE =
    MoveControl_ControlMode_POSE_RELATIVE;
  static constexpr ControlMode VEL_ABSOLUBE =
    MoveControl_ControlMode_VEL_ABSOLUBE;
  static constexpr ControlMode VEL_RELATIVE =
    MoveControl_ControlMode_VEL_RELATIVE;
  static inline bool ControlMode_IsValid(int value) {
    return MoveControl_ControlMode_IsValid(value);
  }
  static constexpr ControlMode ControlMode_MIN =
    MoveControl_ControlMode_ControlMode_MIN;
  static constexpr ControlMode ControlMode_MAX =
    MoveControl_ControlMode_ControlMode_MAX;
  static constexpr int ControlMode_ARRAYSIZE =
    MoveControl_ControlMode_ControlMode_ARRAYSIZE;
  static inline const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor*
  ControlMode_descriptor() {
    return MoveControl_ControlMode_descriptor();
  }
  template<typename T>
  static inline const std::string& ControlMode_Name(T enum_t_value) {
    static_assert(::std::is_same<T, ControlMode>::value ||
      ::std::is_integral<T>::value,
      "Incorrect type passed to function ControlMode_Name.");
    return MoveControl_ControlMode_Name(enum_t_value);
  }
  static inline bool ControlMode_Parse(::PROTOBUF_NAMESPACE_ID::ConstStringParam name,
      ControlMode* value) {
    return MoveControl_ControlMode_Parse(name, value);
  }

  // accessors -------------------------------------------------------

  enum : int {
    kCmdPoseFieldNumber = 2,
    kCmdVelFieldNumber = 3,
    kModeFieldNumber = 1,
    kDurationFieldNumber = 4,
  };
  // .dtproto.geometry_msgs.Pose3d cmd_pose = 2;
  bool has_cmd_pose() const;
  private:
  bool _internal_has_cmd_pose() const;
  public:
  void clear_cmd_pose();
  const ::dtproto::geometry_msgs::Pose3d& cmd_pose() const;
  PROTOBUF_NODISCARD ::dtproto::geometry_msgs::Pose3d* release_cmd_pose();
  ::dtproto::geometry_msgs::Pose3d* mutable_cmd_pose();
  void set_allocated_cmd_pose(::dtproto::geometry_msgs::Pose3d* cmd_pose);
  private:
  const ::dtproto::geometry_msgs::Pose3d& _internal_cmd_pose() const;
  ::dtproto::geometry_msgs::Pose3d* _internal_mutable_cmd_pose();
  public:
  void unsafe_arena_set_allocated_cmd_pose(
      ::dtproto::geometry_msgs::Pose3d* cmd_pose);
  ::dtproto::geometry_msgs::Pose3d* unsafe_arena_release_cmd_pose();

  // .dtproto.geometry_msgs.Twist cmd_vel = 3;
  bool has_cmd_vel() const;
  private:
  bool _internal_has_cmd_vel() const;
  public:
  void clear_cmd_vel();
  const ::dtproto::geometry_msgs::Twist& cmd_vel() const;
  PROTOBUF_NODISCARD ::dtproto::geometry_msgs::Twist* release_cmd_vel();
  ::dtproto::geometry_msgs::Twist* mutable_cmd_vel();
  void set_allocated_cmd_vel(::dtproto::geometry_msgs::Twist* cmd_vel);
  private:
  const ::dtproto::geometry_msgs::Twist& _internal_cmd_vel() const;
  ::dtproto::geometry_msgs::Twist* _internal_mutable_cmd_vel();
  public:
  void unsafe_arena_set_allocated_cmd_vel(
      ::dtproto::geometry_msgs::Twist* cmd_vel);
  ::dtproto::geometry_msgs::Twist* unsafe_arena_release_cmd_vel();

  // .dtproto.robot_msgs.MoveControl.ControlMode mode = 1;
  void clear_mode();
  ::dtproto::robot_msgs::MoveControl_ControlMode mode() const;
  void set_mode(::dtproto::robot_msgs::MoveControl_ControlMode value);
  private:
  ::dtproto::robot_msgs::MoveControl_ControlMode _internal_mode() const;
  void _internal_set_mode(::dtproto::robot_msgs::MoveControl_ControlMode value);
  public:

  // float duration = 4;
  void clear_duration();
  float duration() const;
  void set_duration(float value);
  private:
  float _internal_duration() const;
  void _internal_set_duration(float value);
  public:

  // @@protoc_insertion_point(class_scope:dtproto.robot_msgs.MoveControl)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  struct Impl_ {
    ::dtproto::geometry_msgs::Pose3d* cmd_pose_;
    ::dtproto::geometry_msgs::Twist* cmd_vel_;
    int mode_;
    float duration_;
    mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  };
  union { Impl_ _impl_; };
  friend struct ::TableStruct_dtProto_2frobot_5fmsgs_2fMoveControl_2eproto;
};
// -------------------------------------------------------------------

class MoveControlTimeStamped final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:dtproto.robot_msgs.MoveControlTimeStamped) */ {
 public:
  inline MoveControlTimeStamped() : MoveControlTimeStamped(nullptr) {}
  ~MoveControlTimeStamped() override;
  explicit PROTOBUF_CONSTEXPR MoveControlTimeStamped(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  MoveControlTimeStamped(const MoveControlTimeStamped& from);
  MoveControlTimeStamped(MoveControlTimeStamped&& from) noexcept
    : MoveControlTimeStamped() {
    *this = ::std::move(from);
  }

  inline MoveControlTimeStamped& operator=(const MoveControlTimeStamped& from) {
    CopyFrom(from);
    return *this;
  }
  inline MoveControlTimeStamped& operator=(MoveControlTimeStamped&& from) noexcept {
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
  static const MoveControlTimeStamped& default_instance() {
    return *internal_default_instance();
  }
  static inline const MoveControlTimeStamped* internal_default_instance() {
    return reinterpret_cast<const MoveControlTimeStamped*>(
               &_MoveControlTimeStamped_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(MoveControlTimeStamped& a, MoveControlTimeStamped& b) {
    a.Swap(&b);
  }
  inline void Swap(MoveControlTimeStamped* other) {
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
  void UnsafeArenaSwap(MoveControlTimeStamped* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  MoveControlTimeStamped* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<MoveControlTimeStamped>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const MoveControlTimeStamped& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom( const MoveControlTimeStamped& from) {
    MoveControlTimeStamped::MergeImpl(*this, from);
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
  void InternalSwap(MoveControlTimeStamped* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "dtproto.robot_msgs.MoveControlTimeStamped";
  }
  protected:
  explicit MoveControlTimeStamped(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kHeaderFieldNumber = 1,
    kCmdFieldNumber = 2,
  };
  // .dtproto.std_msgs.Header header = 1;
  bool has_header() const;
  private:
  bool _internal_has_header() const;
  public:
  void clear_header();
  const ::dtproto::std_msgs::Header& header() const;
  PROTOBUF_NODISCARD ::dtproto::std_msgs::Header* release_header();
  ::dtproto::std_msgs::Header* mutable_header();
  void set_allocated_header(::dtproto::std_msgs::Header* header);
  private:
  const ::dtproto::std_msgs::Header& _internal_header() const;
  ::dtproto::std_msgs::Header* _internal_mutable_header();
  public:
  void unsafe_arena_set_allocated_header(
      ::dtproto::std_msgs::Header* header);
  ::dtproto::std_msgs::Header* unsafe_arena_release_header();

  // .dtproto.robot_msgs.MoveControl cmd = 2;
  bool has_cmd() const;
  private:
  bool _internal_has_cmd() const;
  public:
  void clear_cmd();
  const ::dtproto::robot_msgs::MoveControl& cmd() const;
  PROTOBUF_NODISCARD ::dtproto::robot_msgs::MoveControl* release_cmd();
  ::dtproto::robot_msgs::MoveControl* mutable_cmd();
  void set_allocated_cmd(::dtproto::robot_msgs::MoveControl* cmd);
  private:
  const ::dtproto::robot_msgs::MoveControl& _internal_cmd() const;
  ::dtproto::robot_msgs::MoveControl* _internal_mutable_cmd();
  public:
  void unsafe_arena_set_allocated_cmd(
      ::dtproto::robot_msgs::MoveControl* cmd);
  ::dtproto::robot_msgs::MoveControl* unsafe_arena_release_cmd();

  // @@protoc_insertion_point(class_scope:dtproto.robot_msgs.MoveControlTimeStamped)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  struct Impl_ {
    ::dtproto::std_msgs::Header* header_;
    ::dtproto::robot_msgs::MoveControl* cmd_;
    mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  };
  union { Impl_ _impl_; };
  friend struct ::TableStruct_dtProto_2frobot_5fmsgs_2fMoveControl_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// MoveControl

// .dtproto.robot_msgs.MoveControl.ControlMode mode = 1;
inline void MoveControl::clear_mode() {
  _impl_.mode_ = 0;
}
inline ::dtproto::robot_msgs::MoveControl_ControlMode MoveControl::_internal_mode() const {
  return static_cast< ::dtproto::robot_msgs::MoveControl_ControlMode >(_impl_.mode_);
}
inline ::dtproto::robot_msgs::MoveControl_ControlMode MoveControl::mode() const {
  // @@protoc_insertion_point(field_get:dtproto.robot_msgs.MoveControl.mode)
  return _internal_mode();
}
inline void MoveControl::_internal_set_mode(::dtproto::robot_msgs::MoveControl_ControlMode value) {
  
  _impl_.mode_ = value;
}
inline void MoveControl::set_mode(::dtproto::robot_msgs::MoveControl_ControlMode value) {
  _internal_set_mode(value);
  // @@protoc_insertion_point(field_set:dtproto.robot_msgs.MoveControl.mode)
}

// .dtproto.geometry_msgs.Pose3d cmd_pose = 2;
inline bool MoveControl::_internal_has_cmd_pose() const {
  return this != internal_default_instance() && _impl_.cmd_pose_ != nullptr;
}
inline bool MoveControl::has_cmd_pose() const {
  return _internal_has_cmd_pose();
}
inline const ::dtproto::geometry_msgs::Pose3d& MoveControl::_internal_cmd_pose() const {
  const ::dtproto::geometry_msgs::Pose3d* p = _impl_.cmd_pose_;
  return p != nullptr ? *p : reinterpret_cast<const ::dtproto::geometry_msgs::Pose3d&>(
      ::dtproto::geometry_msgs::_Pose3d_default_instance_);
}
inline const ::dtproto::geometry_msgs::Pose3d& MoveControl::cmd_pose() const {
  // @@protoc_insertion_point(field_get:dtproto.robot_msgs.MoveControl.cmd_pose)
  return _internal_cmd_pose();
}
inline void MoveControl::unsafe_arena_set_allocated_cmd_pose(
    ::dtproto::geometry_msgs::Pose3d* cmd_pose) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.cmd_pose_);
  }
  _impl_.cmd_pose_ = cmd_pose;
  if (cmd_pose) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:dtproto.robot_msgs.MoveControl.cmd_pose)
}
inline ::dtproto::geometry_msgs::Pose3d* MoveControl::release_cmd_pose() {
  
  ::dtproto::geometry_msgs::Pose3d* temp = _impl_.cmd_pose_;
  _impl_.cmd_pose_ = nullptr;
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
inline ::dtproto::geometry_msgs::Pose3d* MoveControl::unsafe_arena_release_cmd_pose() {
  // @@protoc_insertion_point(field_release:dtproto.robot_msgs.MoveControl.cmd_pose)
  
  ::dtproto::geometry_msgs::Pose3d* temp = _impl_.cmd_pose_;
  _impl_.cmd_pose_ = nullptr;
  return temp;
}
inline ::dtproto::geometry_msgs::Pose3d* MoveControl::_internal_mutable_cmd_pose() {
  
  if (_impl_.cmd_pose_ == nullptr) {
    auto* p = CreateMaybeMessage<::dtproto::geometry_msgs::Pose3d>(GetArenaForAllocation());
    _impl_.cmd_pose_ = p;
  }
  return _impl_.cmd_pose_;
}
inline ::dtproto::geometry_msgs::Pose3d* MoveControl::mutable_cmd_pose() {
  ::dtproto::geometry_msgs::Pose3d* _msg = _internal_mutable_cmd_pose();
  // @@protoc_insertion_point(field_mutable:dtproto.robot_msgs.MoveControl.cmd_pose)
  return _msg;
}
inline void MoveControl::set_allocated_cmd_pose(::dtproto::geometry_msgs::Pose3d* cmd_pose) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.cmd_pose_);
  }
  if (cmd_pose) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalGetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(cmd_pose));
    if (message_arena != submessage_arena) {
      cmd_pose = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, cmd_pose, submessage_arena);
    }
    
  } else {
    
  }
  _impl_.cmd_pose_ = cmd_pose;
  // @@protoc_insertion_point(field_set_allocated:dtproto.robot_msgs.MoveControl.cmd_pose)
}

// .dtproto.geometry_msgs.Twist cmd_vel = 3;
inline bool MoveControl::_internal_has_cmd_vel() const {
  return this != internal_default_instance() && _impl_.cmd_vel_ != nullptr;
}
inline bool MoveControl::has_cmd_vel() const {
  return _internal_has_cmd_vel();
}
inline const ::dtproto::geometry_msgs::Twist& MoveControl::_internal_cmd_vel() const {
  const ::dtproto::geometry_msgs::Twist* p = _impl_.cmd_vel_;
  return p != nullptr ? *p : reinterpret_cast<const ::dtproto::geometry_msgs::Twist&>(
      ::dtproto::geometry_msgs::_Twist_default_instance_);
}
inline const ::dtproto::geometry_msgs::Twist& MoveControl::cmd_vel() const {
  // @@protoc_insertion_point(field_get:dtproto.robot_msgs.MoveControl.cmd_vel)
  return _internal_cmd_vel();
}
inline void MoveControl::unsafe_arena_set_allocated_cmd_vel(
    ::dtproto::geometry_msgs::Twist* cmd_vel) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.cmd_vel_);
  }
  _impl_.cmd_vel_ = cmd_vel;
  if (cmd_vel) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:dtproto.robot_msgs.MoveControl.cmd_vel)
}
inline ::dtproto::geometry_msgs::Twist* MoveControl::release_cmd_vel() {
  
  ::dtproto::geometry_msgs::Twist* temp = _impl_.cmd_vel_;
  _impl_.cmd_vel_ = nullptr;
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
inline ::dtproto::geometry_msgs::Twist* MoveControl::unsafe_arena_release_cmd_vel() {
  // @@protoc_insertion_point(field_release:dtproto.robot_msgs.MoveControl.cmd_vel)
  
  ::dtproto::geometry_msgs::Twist* temp = _impl_.cmd_vel_;
  _impl_.cmd_vel_ = nullptr;
  return temp;
}
inline ::dtproto::geometry_msgs::Twist* MoveControl::_internal_mutable_cmd_vel() {
  
  if (_impl_.cmd_vel_ == nullptr) {
    auto* p = CreateMaybeMessage<::dtproto::geometry_msgs::Twist>(GetArenaForAllocation());
    _impl_.cmd_vel_ = p;
  }
  return _impl_.cmd_vel_;
}
inline ::dtproto::geometry_msgs::Twist* MoveControl::mutable_cmd_vel() {
  ::dtproto::geometry_msgs::Twist* _msg = _internal_mutable_cmd_vel();
  // @@protoc_insertion_point(field_mutable:dtproto.robot_msgs.MoveControl.cmd_vel)
  return _msg;
}
inline void MoveControl::set_allocated_cmd_vel(::dtproto::geometry_msgs::Twist* cmd_vel) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.cmd_vel_);
  }
  if (cmd_vel) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalGetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(cmd_vel));
    if (message_arena != submessage_arena) {
      cmd_vel = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, cmd_vel, submessage_arena);
    }
    
  } else {
    
  }
  _impl_.cmd_vel_ = cmd_vel;
  // @@protoc_insertion_point(field_set_allocated:dtproto.robot_msgs.MoveControl.cmd_vel)
}

// float duration = 4;
inline void MoveControl::clear_duration() {
  _impl_.duration_ = 0;
}
inline float MoveControl::_internal_duration() const {
  return _impl_.duration_;
}
inline float MoveControl::duration() const {
  // @@protoc_insertion_point(field_get:dtproto.robot_msgs.MoveControl.duration)
  return _internal_duration();
}
inline void MoveControl::_internal_set_duration(float value) {
  
  _impl_.duration_ = value;
}
inline void MoveControl::set_duration(float value) {
  _internal_set_duration(value);
  // @@protoc_insertion_point(field_set:dtproto.robot_msgs.MoveControl.duration)
}

// -------------------------------------------------------------------

// MoveControlTimeStamped

// .dtproto.std_msgs.Header header = 1;
inline bool MoveControlTimeStamped::_internal_has_header() const {
  return this != internal_default_instance() && _impl_.header_ != nullptr;
}
inline bool MoveControlTimeStamped::has_header() const {
  return _internal_has_header();
}
inline const ::dtproto::std_msgs::Header& MoveControlTimeStamped::_internal_header() const {
  const ::dtproto::std_msgs::Header* p = _impl_.header_;
  return p != nullptr ? *p : reinterpret_cast<const ::dtproto::std_msgs::Header&>(
      ::dtproto::std_msgs::_Header_default_instance_);
}
inline const ::dtproto::std_msgs::Header& MoveControlTimeStamped::header() const {
  // @@protoc_insertion_point(field_get:dtproto.robot_msgs.MoveControlTimeStamped.header)
  return _internal_header();
}
inline void MoveControlTimeStamped::unsafe_arena_set_allocated_header(
    ::dtproto::std_msgs::Header* header) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.header_);
  }
  _impl_.header_ = header;
  if (header) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:dtproto.robot_msgs.MoveControlTimeStamped.header)
}
inline ::dtproto::std_msgs::Header* MoveControlTimeStamped::release_header() {
  
  ::dtproto::std_msgs::Header* temp = _impl_.header_;
  _impl_.header_ = nullptr;
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
inline ::dtproto::std_msgs::Header* MoveControlTimeStamped::unsafe_arena_release_header() {
  // @@protoc_insertion_point(field_release:dtproto.robot_msgs.MoveControlTimeStamped.header)
  
  ::dtproto::std_msgs::Header* temp = _impl_.header_;
  _impl_.header_ = nullptr;
  return temp;
}
inline ::dtproto::std_msgs::Header* MoveControlTimeStamped::_internal_mutable_header() {
  
  if (_impl_.header_ == nullptr) {
    auto* p = CreateMaybeMessage<::dtproto::std_msgs::Header>(GetArenaForAllocation());
    _impl_.header_ = p;
  }
  return _impl_.header_;
}
inline ::dtproto::std_msgs::Header* MoveControlTimeStamped::mutable_header() {
  ::dtproto::std_msgs::Header* _msg = _internal_mutable_header();
  // @@protoc_insertion_point(field_mutable:dtproto.robot_msgs.MoveControlTimeStamped.header)
  return _msg;
}
inline void MoveControlTimeStamped::set_allocated_header(::dtproto::std_msgs::Header* header) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.header_);
  }
  if (header) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalGetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(header));
    if (message_arena != submessage_arena) {
      header = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, header, submessage_arena);
    }
    
  } else {
    
  }
  _impl_.header_ = header;
  // @@protoc_insertion_point(field_set_allocated:dtproto.robot_msgs.MoveControlTimeStamped.header)
}

// .dtproto.robot_msgs.MoveControl cmd = 2;
inline bool MoveControlTimeStamped::_internal_has_cmd() const {
  return this != internal_default_instance() && _impl_.cmd_ != nullptr;
}
inline bool MoveControlTimeStamped::has_cmd() const {
  return _internal_has_cmd();
}
inline void MoveControlTimeStamped::clear_cmd() {
  if (GetArenaForAllocation() == nullptr && _impl_.cmd_ != nullptr) {
    delete _impl_.cmd_;
  }
  _impl_.cmd_ = nullptr;
}
inline const ::dtproto::robot_msgs::MoveControl& MoveControlTimeStamped::_internal_cmd() const {
  const ::dtproto::robot_msgs::MoveControl* p = _impl_.cmd_;
  return p != nullptr ? *p : reinterpret_cast<const ::dtproto::robot_msgs::MoveControl&>(
      ::dtproto::robot_msgs::_MoveControl_default_instance_);
}
inline const ::dtproto::robot_msgs::MoveControl& MoveControlTimeStamped::cmd() const {
  // @@protoc_insertion_point(field_get:dtproto.robot_msgs.MoveControlTimeStamped.cmd)
  return _internal_cmd();
}
inline void MoveControlTimeStamped::unsafe_arena_set_allocated_cmd(
    ::dtproto::robot_msgs::MoveControl* cmd) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.cmd_);
  }
  _impl_.cmd_ = cmd;
  if (cmd) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:dtproto.robot_msgs.MoveControlTimeStamped.cmd)
}
inline ::dtproto::robot_msgs::MoveControl* MoveControlTimeStamped::release_cmd() {
  
  ::dtproto::robot_msgs::MoveControl* temp = _impl_.cmd_;
  _impl_.cmd_ = nullptr;
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
inline ::dtproto::robot_msgs::MoveControl* MoveControlTimeStamped::unsafe_arena_release_cmd() {
  // @@protoc_insertion_point(field_release:dtproto.robot_msgs.MoveControlTimeStamped.cmd)
  
  ::dtproto::robot_msgs::MoveControl* temp = _impl_.cmd_;
  _impl_.cmd_ = nullptr;
  return temp;
}
inline ::dtproto::robot_msgs::MoveControl* MoveControlTimeStamped::_internal_mutable_cmd() {
  
  if (_impl_.cmd_ == nullptr) {
    auto* p = CreateMaybeMessage<::dtproto::robot_msgs::MoveControl>(GetArenaForAllocation());
    _impl_.cmd_ = p;
  }
  return _impl_.cmd_;
}
inline ::dtproto::robot_msgs::MoveControl* MoveControlTimeStamped::mutable_cmd() {
  ::dtproto::robot_msgs::MoveControl* _msg = _internal_mutable_cmd();
  // @@protoc_insertion_point(field_mutable:dtproto.robot_msgs.MoveControlTimeStamped.cmd)
  return _msg;
}
inline void MoveControlTimeStamped::set_allocated_cmd(::dtproto::robot_msgs::MoveControl* cmd) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete _impl_.cmd_;
  }
  if (cmd) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalGetOwningArena(cmd);
    if (message_arena != submessage_arena) {
      cmd = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, cmd, submessage_arena);
    }
    
  } else {
    
  }
  _impl_.cmd_ = cmd;
  // @@protoc_insertion_point(field_set_allocated:dtproto.robot_msgs.MoveControlTimeStamped.cmd)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace robot_msgs
}  // namespace dtproto

PROTOBUF_NAMESPACE_OPEN

template <> struct is_proto_enum< ::dtproto::robot_msgs::MoveControl_ControlMode> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::dtproto::robot_msgs::MoveControl_ControlMode>() {
  return ::dtproto::robot_msgs::MoveControl_ControlMode_descriptor();
}

PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_dtProto_2frobot_5fmsgs_2fMoveControl_2eproto