// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: dtProto/robot_msgs/RobotState.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_dtProto_2frobot_5fmsgs_2fRobotState_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_dtProto_2frobot_5fmsgs_2fRobotState_2eproto

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
#include "dtProto/std_msgs/Header.pb.h"
#include "dtProto/geometry_msgs/Point.pb.h"
#include "dtProto/geometry_msgs/Vector.pb.h"
#include "dtProto/geometry_msgs/Matrix.pb.h"
#include "dtProto/geometry_msgs/Pose.pb.h"
#include "dtProto/geometry_msgs/Twist.pb.h"
#include "dtProto/geometry_msgs/Orientation.pb.h"
#include "dtProto/sensor_msgs/JointState.pb.h"
#include "dtProto/sensor_msgs/BatteryState.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_dtProto_2frobot_5fmsgs_2fRobotState_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_dtProto_2frobot_5fmsgs_2fRobotState_2eproto {
  static const uint32_t offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_dtProto_2frobot_5fmsgs_2fRobotState_2eproto;
namespace dtproto {
namespace robot_msgs {
class RobotState;
struct RobotStateDefaultTypeInternal;
extern RobotStateDefaultTypeInternal _RobotState_default_instance_;
class RobotStateTimeStamped;
struct RobotStateTimeStampedDefaultTypeInternal;
extern RobotStateTimeStampedDefaultTypeInternal _RobotStateTimeStamped_default_instance_;
}  // namespace robot_msgs
}  // namespace dtproto
PROTOBUF_NAMESPACE_OPEN
template<> ::dtproto::robot_msgs::RobotState* Arena::CreateMaybeMessage<::dtproto::robot_msgs::RobotState>(Arena*);
template<> ::dtproto::robot_msgs::RobotStateTimeStamped* Arena::CreateMaybeMessage<::dtproto::robot_msgs::RobotStateTimeStamped>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace dtproto {
namespace robot_msgs {

// ===================================================================

class RobotState final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:dtproto.robot_msgs.RobotState) */ {
 public:
  inline RobotState() : RobotState(nullptr) {}
  ~RobotState() override;
  explicit PROTOBUF_CONSTEXPR RobotState(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  RobotState(const RobotState& from);
  RobotState(RobotState&& from) noexcept
    : RobotState() {
    *this = ::std::move(from);
  }

  inline RobotState& operator=(const RobotState& from) {
    CopyFrom(from);
    return *this;
  }
  inline RobotState& operator=(RobotState&& from) noexcept {
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
  static const RobotState& default_instance() {
    return *internal_default_instance();
  }
  static inline const RobotState* internal_default_instance() {
    return reinterpret_cast<const RobotState*>(
               &_RobotState_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(RobotState& a, RobotState& b) {
    a.Swap(&b);
  }
  inline void Swap(RobotState* other) {
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
  void UnsafeArenaSwap(RobotState* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  RobotState* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<RobotState>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const RobotState& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom( const RobotState& from) {
    RobotState::MergeImpl(*this, from);
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
  void InternalSwap(RobotState* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "dtproto.robot_msgs.RobotState";
  }
  protected:
  explicit RobotState(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kJointStateFieldNumber = 3,
    kBasePoseFieldNumber = 1,
    kBaseVelocityFieldNumber = 2,
    kBatteryStateFieldNumber = 4,
    kStatusFieldNumber = 5,
  };
  // repeated .dtproto.sensor_msgs.JointState joint_state = 3;
  int joint_state_size() const;
  private:
  int _internal_joint_state_size() const;
  public:
  void clear_joint_state();
  ::dtproto::sensor_msgs::JointState* mutable_joint_state(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::dtproto::sensor_msgs::JointState >*
      mutable_joint_state();
  private:
  const ::dtproto::sensor_msgs::JointState& _internal_joint_state(int index) const;
  ::dtproto::sensor_msgs::JointState* _internal_add_joint_state();
  public:
  const ::dtproto::sensor_msgs::JointState& joint_state(int index) const;
  ::dtproto::sensor_msgs::JointState* add_joint_state();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::dtproto::sensor_msgs::JointState >&
      joint_state() const;

  // .dtproto.geometry_msgs.Pose3d base_pose = 1;
  bool has_base_pose() const;
  private:
  bool _internal_has_base_pose() const;
  public:
  void clear_base_pose();
  const ::dtproto::geometry_msgs::Pose3d& base_pose() const;
  PROTOBUF_NODISCARD ::dtproto::geometry_msgs::Pose3d* release_base_pose();
  ::dtproto::geometry_msgs::Pose3d* mutable_base_pose();
  void set_allocated_base_pose(::dtproto::geometry_msgs::Pose3d* base_pose);
  private:
  const ::dtproto::geometry_msgs::Pose3d& _internal_base_pose() const;
  ::dtproto::geometry_msgs::Pose3d* _internal_mutable_base_pose();
  public:
  void unsafe_arena_set_allocated_base_pose(
      ::dtproto::geometry_msgs::Pose3d* base_pose);
  ::dtproto::geometry_msgs::Pose3d* unsafe_arena_release_base_pose();

  // .dtproto.geometry_msgs.Twist base_velocity = 2;
  bool has_base_velocity() const;
  private:
  bool _internal_has_base_velocity() const;
  public:
  void clear_base_velocity();
  const ::dtproto::geometry_msgs::Twist& base_velocity() const;
  PROTOBUF_NODISCARD ::dtproto::geometry_msgs::Twist* release_base_velocity();
  ::dtproto::geometry_msgs::Twist* mutable_base_velocity();
  void set_allocated_base_velocity(::dtproto::geometry_msgs::Twist* base_velocity);
  private:
  const ::dtproto::geometry_msgs::Twist& _internal_base_velocity() const;
  ::dtproto::geometry_msgs::Twist* _internal_mutable_base_velocity();
  public:
  void unsafe_arena_set_allocated_base_velocity(
      ::dtproto::geometry_msgs::Twist* base_velocity);
  ::dtproto::geometry_msgs::Twist* unsafe_arena_release_base_velocity();

  // .dtproto.sensor_msgs.BatteryState battery_state = 4;
  bool has_battery_state() const;
  private:
  bool _internal_has_battery_state() const;
  public:
  void clear_battery_state();
  const ::dtproto::sensor_msgs::BatteryState& battery_state() const;
  PROTOBUF_NODISCARD ::dtproto::sensor_msgs::BatteryState* release_battery_state();
  ::dtproto::sensor_msgs::BatteryState* mutable_battery_state();
  void set_allocated_battery_state(::dtproto::sensor_msgs::BatteryState* battery_state);
  private:
  const ::dtproto::sensor_msgs::BatteryState& _internal_battery_state() const;
  ::dtproto::sensor_msgs::BatteryState* _internal_mutable_battery_state();
  public:
  void unsafe_arena_set_allocated_battery_state(
      ::dtproto::sensor_msgs::BatteryState* battery_state);
  ::dtproto::sensor_msgs::BatteryState* unsafe_arena_release_battery_state();

  // uint32 status = 5;
  void clear_status();
  uint32_t status() const;
  void set_status(uint32_t value);
  private:
  uint32_t _internal_status() const;
  void _internal_set_status(uint32_t value);
  public:

  // @@protoc_insertion_point(class_scope:dtproto.robot_msgs.RobotState)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  struct Impl_ {
    ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::dtproto::sensor_msgs::JointState > joint_state_;
    ::dtproto::geometry_msgs::Pose3d* base_pose_;
    ::dtproto::geometry_msgs::Twist* base_velocity_;
    ::dtproto::sensor_msgs::BatteryState* battery_state_;
    uint32_t status_;
    mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  };
  union { Impl_ _impl_; };
  friend struct ::TableStruct_dtProto_2frobot_5fmsgs_2fRobotState_2eproto;
};
// -------------------------------------------------------------------

class RobotStateTimeStamped final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:dtproto.robot_msgs.RobotStateTimeStamped) */ {
 public:
  inline RobotStateTimeStamped() : RobotStateTimeStamped(nullptr) {}
  ~RobotStateTimeStamped() override;
  explicit PROTOBUF_CONSTEXPR RobotStateTimeStamped(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  RobotStateTimeStamped(const RobotStateTimeStamped& from);
  RobotStateTimeStamped(RobotStateTimeStamped&& from) noexcept
    : RobotStateTimeStamped() {
    *this = ::std::move(from);
  }

  inline RobotStateTimeStamped& operator=(const RobotStateTimeStamped& from) {
    CopyFrom(from);
    return *this;
  }
  inline RobotStateTimeStamped& operator=(RobotStateTimeStamped&& from) noexcept {
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
  static const RobotStateTimeStamped& default_instance() {
    return *internal_default_instance();
  }
  static inline const RobotStateTimeStamped* internal_default_instance() {
    return reinterpret_cast<const RobotStateTimeStamped*>(
               &_RobotStateTimeStamped_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(RobotStateTimeStamped& a, RobotStateTimeStamped& b) {
    a.Swap(&b);
  }
  inline void Swap(RobotStateTimeStamped* other) {
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
  void UnsafeArenaSwap(RobotStateTimeStamped* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  RobotStateTimeStamped* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<RobotStateTimeStamped>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const RobotStateTimeStamped& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom( const RobotStateTimeStamped& from) {
    RobotStateTimeStamped::MergeImpl(*this, from);
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
  void InternalSwap(RobotStateTimeStamped* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "dtproto.robot_msgs.RobotStateTimeStamped";
  }
  protected:
  explicit RobotStateTimeStamped(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kHeaderFieldNumber = 1,
    kStateFieldNumber = 2,
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

  // .dtproto.robot_msgs.RobotState state = 2;
  bool has_state() const;
  private:
  bool _internal_has_state() const;
  public:
  void clear_state();
  const ::dtproto::robot_msgs::RobotState& state() const;
  PROTOBUF_NODISCARD ::dtproto::robot_msgs::RobotState* release_state();
  ::dtproto::robot_msgs::RobotState* mutable_state();
  void set_allocated_state(::dtproto::robot_msgs::RobotState* state);
  private:
  const ::dtproto::robot_msgs::RobotState& _internal_state() const;
  ::dtproto::robot_msgs::RobotState* _internal_mutable_state();
  public:
  void unsafe_arena_set_allocated_state(
      ::dtproto::robot_msgs::RobotState* state);
  ::dtproto::robot_msgs::RobotState* unsafe_arena_release_state();

  // @@protoc_insertion_point(class_scope:dtproto.robot_msgs.RobotStateTimeStamped)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  struct Impl_ {
    ::dtproto::std_msgs::Header* header_;
    ::dtproto::robot_msgs::RobotState* state_;
    mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  };
  union { Impl_ _impl_; };
  friend struct ::TableStruct_dtProto_2frobot_5fmsgs_2fRobotState_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// RobotState

// .dtproto.geometry_msgs.Pose3d base_pose = 1;
inline bool RobotState::_internal_has_base_pose() const {
  return this != internal_default_instance() && _impl_.base_pose_ != nullptr;
}
inline bool RobotState::has_base_pose() const {
  return _internal_has_base_pose();
}
inline const ::dtproto::geometry_msgs::Pose3d& RobotState::_internal_base_pose() const {
  const ::dtproto::geometry_msgs::Pose3d* p = _impl_.base_pose_;
  return p != nullptr ? *p : reinterpret_cast<const ::dtproto::geometry_msgs::Pose3d&>(
      ::dtproto::geometry_msgs::_Pose3d_default_instance_);
}
inline const ::dtproto::geometry_msgs::Pose3d& RobotState::base_pose() const {
  // @@protoc_insertion_point(field_get:dtproto.robot_msgs.RobotState.base_pose)
  return _internal_base_pose();
}
inline void RobotState::unsafe_arena_set_allocated_base_pose(
    ::dtproto::geometry_msgs::Pose3d* base_pose) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.base_pose_);
  }
  _impl_.base_pose_ = base_pose;
  if (base_pose) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:dtproto.robot_msgs.RobotState.base_pose)
}
inline ::dtproto::geometry_msgs::Pose3d* RobotState::release_base_pose() {
  
  ::dtproto::geometry_msgs::Pose3d* temp = _impl_.base_pose_;
  _impl_.base_pose_ = nullptr;
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
inline ::dtproto::geometry_msgs::Pose3d* RobotState::unsafe_arena_release_base_pose() {
  // @@protoc_insertion_point(field_release:dtproto.robot_msgs.RobotState.base_pose)
  
  ::dtproto::geometry_msgs::Pose3d* temp = _impl_.base_pose_;
  _impl_.base_pose_ = nullptr;
  return temp;
}
inline ::dtproto::geometry_msgs::Pose3d* RobotState::_internal_mutable_base_pose() {
  
  if (_impl_.base_pose_ == nullptr) {
    auto* p = CreateMaybeMessage<::dtproto::geometry_msgs::Pose3d>(GetArenaForAllocation());
    _impl_.base_pose_ = p;
  }
  return _impl_.base_pose_;
}
inline ::dtproto::geometry_msgs::Pose3d* RobotState::mutable_base_pose() {
  ::dtproto::geometry_msgs::Pose3d* _msg = _internal_mutable_base_pose();
  // @@protoc_insertion_point(field_mutable:dtproto.robot_msgs.RobotState.base_pose)
  return _msg;
}
inline void RobotState::set_allocated_base_pose(::dtproto::geometry_msgs::Pose3d* base_pose) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.base_pose_);
  }
  if (base_pose) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalGetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(base_pose));
    if (message_arena != submessage_arena) {
      base_pose = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, base_pose, submessage_arena);
    }
    
  } else {
    
  }
  _impl_.base_pose_ = base_pose;
  // @@protoc_insertion_point(field_set_allocated:dtproto.robot_msgs.RobotState.base_pose)
}

// .dtproto.geometry_msgs.Twist base_velocity = 2;
inline bool RobotState::_internal_has_base_velocity() const {
  return this != internal_default_instance() && _impl_.base_velocity_ != nullptr;
}
inline bool RobotState::has_base_velocity() const {
  return _internal_has_base_velocity();
}
inline const ::dtproto::geometry_msgs::Twist& RobotState::_internal_base_velocity() const {
  const ::dtproto::geometry_msgs::Twist* p = _impl_.base_velocity_;
  return p != nullptr ? *p : reinterpret_cast<const ::dtproto::geometry_msgs::Twist&>(
      ::dtproto::geometry_msgs::_Twist_default_instance_);
}
inline const ::dtproto::geometry_msgs::Twist& RobotState::base_velocity() const {
  // @@protoc_insertion_point(field_get:dtproto.robot_msgs.RobotState.base_velocity)
  return _internal_base_velocity();
}
inline void RobotState::unsafe_arena_set_allocated_base_velocity(
    ::dtproto::geometry_msgs::Twist* base_velocity) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.base_velocity_);
  }
  _impl_.base_velocity_ = base_velocity;
  if (base_velocity) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:dtproto.robot_msgs.RobotState.base_velocity)
}
inline ::dtproto::geometry_msgs::Twist* RobotState::release_base_velocity() {
  
  ::dtproto::geometry_msgs::Twist* temp = _impl_.base_velocity_;
  _impl_.base_velocity_ = nullptr;
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
inline ::dtproto::geometry_msgs::Twist* RobotState::unsafe_arena_release_base_velocity() {
  // @@protoc_insertion_point(field_release:dtproto.robot_msgs.RobotState.base_velocity)
  
  ::dtproto::geometry_msgs::Twist* temp = _impl_.base_velocity_;
  _impl_.base_velocity_ = nullptr;
  return temp;
}
inline ::dtproto::geometry_msgs::Twist* RobotState::_internal_mutable_base_velocity() {
  
  if (_impl_.base_velocity_ == nullptr) {
    auto* p = CreateMaybeMessage<::dtproto::geometry_msgs::Twist>(GetArenaForAllocation());
    _impl_.base_velocity_ = p;
  }
  return _impl_.base_velocity_;
}
inline ::dtproto::geometry_msgs::Twist* RobotState::mutable_base_velocity() {
  ::dtproto::geometry_msgs::Twist* _msg = _internal_mutable_base_velocity();
  // @@protoc_insertion_point(field_mutable:dtproto.robot_msgs.RobotState.base_velocity)
  return _msg;
}
inline void RobotState::set_allocated_base_velocity(::dtproto::geometry_msgs::Twist* base_velocity) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.base_velocity_);
  }
  if (base_velocity) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalGetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(base_velocity));
    if (message_arena != submessage_arena) {
      base_velocity = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, base_velocity, submessage_arena);
    }
    
  } else {
    
  }
  _impl_.base_velocity_ = base_velocity;
  // @@protoc_insertion_point(field_set_allocated:dtproto.robot_msgs.RobotState.base_velocity)
}

// repeated .dtproto.sensor_msgs.JointState joint_state = 3;
inline int RobotState::_internal_joint_state_size() const {
  return _impl_.joint_state_.size();
}
inline int RobotState::joint_state_size() const {
  return _internal_joint_state_size();
}
inline ::dtproto::sensor_msgs::JointState* RobotState::mutable_joint_state(int index) {
  // @@protoc_insertion_point(field_mutable:dtproto.robot_msgs.RobotState.joint_state)
  return _impl_.joint_state_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::dtproto::sensor_msgs::JointState >*
RobotState::mutable_joint_state() {
  // @@protoc_insertion_point(field_mutable_list:dtproto.robot_msgs.RobotState.joint_state)
  return &_impl_.joint_state_;
}
inline const ::dtproto::sensor_msgs::JointState& RobotState::_internal_joint_state(int index) const {
  return _impl_.joint_state_.Get(index);
}
inline const ::dtproto::sensor_msgs::JointState& RobotState::joint_state(int index) const {
  // @@protoc_insertion_point(field_get:dtproto.robot_msgs.RobotState.joint_state)
  return _internal_joint_state(index);
}
inline ::dtproto::sensor_msgs::JointState* RobotState::_internal_add_joint_state() {
  return _impl_.joint_state_.Add();
}
inline ::dtproto::sensor_msgs::JointState* RobotState::add_joint_state() {
  ::dtproto::sensor_msgs::JointState* _add = _internal_add_joint_state();
  // @@protoc_insertion_point(field_add:dtproto.robot_msgs.RobotState.joint_state)
  return _add;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::dtproto::sensor_msgs::JointState >&
RobotState::joint_state() const {
  // @@protoc_insertion_point(field_list:dtproto.robot_msgs.RobotState.joint_state)
  return _impl_.joint_state_;
}

// .dtproto.sensor_msgs.BatteryState battery_state = 4;
inline bool RobotState::_internal_has_battery_state() const {
  return this != internal_default_instance() && _impl_.battery_state_ != nullptr;
}
inline bool RobotState::has_battery_state() const {
  return _internal_has_battery_state();
}
inline const ::dtproto::sensor_msgs::BatteryState& RobotState::_internal_battery_state() const {
  const ::dtproto::sensor_msgs::BatteryState* p = _impl_.battery_state_;
  return p != nullptr ? *p : reinterpret_cast<const ::dtproto::sensor_msgs::BatteryState&>(
      ::dtproto::sensor_msgs::_BatteryState_default_instance_);
}
inline const ::dtproto::sensor_msgs::BatteryState& RobotState::battery_state() const {
  // @@protoc_insertion_point(field_get:dtproto.robot_msgs.RobotState.battery_state)
  return _internal_battery_state();
}
inline void RobotState::unsafe_arena_set_allocated_battery_state(
    ::dtproto::sensor_msgs::BatteryState* battery_state) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.battery_state_);
  }
  _impl_.battery_state_ = battery_state;
  if (battery_state) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:dtproto.robot_msgs.RobotState.battery_state)
}
inline ::dtproto::sensor_msgs::BatteryState* RobotState::release_battery_state() {
  
  ::dtproto::sensor_msgs::BatteryState* temp = _impl_.battery_state_;
  _impl_.battery_state_ = nullptr;
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
inline ::dtproto::sensor_msgs::BatteryState* RobotState::unsafe_arena_release_battery_state() {
  // @@protoc_insertion_point(field_release:dtproto.robot_msgs.RobotState.battery_state)
  
  ::dtproto::sensor_msgs::BatteryState* temp = _impl_.battery_state_;
  _impl_.battery_state_ = nullptr;
  return temp;
}
inline ::dtproto::sensor_msgs::BatteryState* RobotState::_internal_mutable_battery_state() {
  
  if (_impl_.battery_state_ == nullptr) {
    auto* p = CreateMaybeMessage<::dtproto::sensor_msgs::BatteryState>(GetArenaForAllocation());
    _impl_.battery_state_ = p;
  }
  return _impl_.battery_state_;
}
inline ::dtproto::sensor_msgs::BatteryState* RobotState::mutable_battery_state() {
  ::dtproto::sensor_msgs::BatteryState* _msg = _internal_mutable_battery_state();
  // @@protoc_insertion_point(field_mutable:dtproto.robot_msgs.RobotState.battery_state)
  return _msg;
}
inline void RobotState::set_allocated_battery_state(::dtproto::sensor_msgs::BatteryState* battery_state) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.battery_state_);
  }
  if (battery_state) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalGetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(battery_state));
    if (message_arena != submessage_arena) {
      battery_state = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, battery_state, submessage_arena);
    }
    
  } else {
    
  }
  _impl_.battery_state_ = battery_state;
  // @@protoc_insertion_point(field_set_allocated:dtproto.robot_msgs.RobotState.battery_state)
}

// uint32 status = 5;
inline void RobotState::clear_status() {
  _impl_.status_ = 0u;
}
inline uint32_t RobotState::_internal_status() const {
  return _impl_.status_;
}
inline uint32_t RobotState::status() const {
  // @@protoc_insertion_point(field_get:dtproto.robot_msgs.RobotState.status)
  return _internal_status();
}
inline void RobotState::_internal_set_status(uint32_t value) {
  
  _impl_.status_ = value;
}
inline void RobotState::set_status(uint32_t value) {
  _internal_set_status(value);
  // @@protoc_insertion_point(field_set:dtproto.robot_msgs.RobotState.status)
}

// -------------------------------------------------------------------

// RobotStateTimeStamped

// .dtproto.std_msgs.Header header = 1;
inline bool RobotStateTimeStamped::_internal_has_header() const {
  return this != internal_default_instance() && _impl_.header_ != nullptr;
}
inline bool RobotStateTimeStamped::has_header() const {
  return _internal_has_header();
}
inline const ::dtproto::std_msgs::Header& RobotStateTimeStamped::_internal_header() const {
  const ::dtproto::std_msgs::Header* p = _impl_.header_;
  return p != nullptr ? *p : reinterpret_cast<const ::dtproto::std_msgs::Header&>(
      ::dtproto::std_msgs::_Header_default_instance_);
}
inline const ::dtproto::std_msgs::Header& RobotStateTimeStamped::header() const {
  // @@protoc_insertion_point(field_get:dtproto.robot_msgs.RobotStateTimeStamped.header)
  return _internal_header();
}
inline void RobotStateTimeStamped::unsafe_arena_set_allocated_header(
    ::dtproto::std_msgs::Header* header) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.header_);
  }
  _impl_.header_ = header;
  if (header) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:dtproto.robot_msgs.RobotStateTimeStamped.header)
}
inline ::dtproto::std_msgs::Header* RobotStateTimeStamped::release_header() {
  
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
inline ::dtproto::std_msgs::Header* RobotStateTimeStamped::unsafe_arena_release_header() {
  // @@protoc_insertion_point(field_release:dtproto.robot_msgs.RobotStateTimeStamped.header)
  
  ::dtproto::std_msgs::Header* temp = _impl_.header_;
  _impl_.header_ = nullptr;
  return temp;
}
inline ::dtproto::std_msgs::Header* RobotStateTimeStamped::_internal_mutable_header() {
  
  if (_impl_.header_ == nullptr) {
    auto* p = CreateMaybeMessage<::dtproto::std_msgs::Header>(GetArenaForAllocation());
    _impl_.header_ = p;
  }
  return _impl_.header_;
}
inline ::dtproto::std_msgs::Header* RobotStateTimeStamped::mutable_header() {
  ::dtproto::std_msgs::Header* _msg = _internal_mutable_header();
  // @@protoc_insertion_point(field_mutable:dtproto.robot_msgs.RobotStateTimeStamped.header)
  return _msg;
}
inline void RobotStateTimeStamped::set_allocated_header(::dtproto::std_msgs::Header* header) {
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
  // @@protoc_insertion_point(field_set_allocated:dtproto.robot_msgs.RobotStateTimeStamped.header)
}

// .dtproto.robot_msgs.RobotState state = 2;
inline bool RobotStateTimeStamped::_internal_has_state() const {
  return this != internal_default_instance() && _impl_.state_ != nullptr;
}
inline bool RobotStateTimeStamped::has_state() const {
  return _internal_has_state();
}
inline void RobotStateTimeStamped::clear_state() {
  if (GetArenaForAllocation() == nullptr && _impl_.state_ != nullptr) {
    delete _impl_.state_;
  }
  _impl_.state_ = nullptr;
}
inline const ::dtproto::robot_msgs::RobotState& RobotStateTimeStamped::_internal_state() const {
  const ::dtproto::robot_msgs::RobotState* p = _impl_.state_;
  return p != nullptr ? *p : reinterpret_cast<const ::dtproto::robot_msgs::RobotState&>(
      ::dtproto::robot_msgs::_RobotState_default_instance_);
}
inline const ::dtproto::robot_msgs::RobotState& RobotStateTimeStamped::state() const {
  // @@protoc_insertion_point(field_get:dtproto.robot_msgs.RobotStateTimeStamped.state)
  return _internal_state();
}
inline void RobotStateTimeStamped::unsafe_arena_set_allocated_state(
    ::dtproto::robot_msgs::RobotState* state) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.state_);
  }
  _impl_.state_ = state;
  if (state) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:dtproto.robot_msgs.RobotStateTimeStamped.state)
}
inline ::dtproto::robot_msgs::RobotState* RobotStateTimeStamped::release_state() {
  
  ::dtproto::robot_msgs::RobotState* temp = _impl_.state_;
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
inline ::dtproto::robot_msgs::RobotState* RobotStateTimeStamped::unsafe_arena_release_state() {
  // @@protoc_insertion_point(field_release:dtproto.robot_msgs.RobotStateTimeStamped.state)
  
  ::dtproto::robot_msgs::RobotState* temp = _impl_.state_;
  _impl_.state_ = nullptr;
  return temp;
}
inline ::dtproto::robot_msgs::RobotState* RobotStateTimeStamped::_internal_mutable_state() {
  
  if (_impl_.state_ == nullptr) {
    auto* p = CreateMaybeMessage<::dtproto::robot_msgs::RobotState>(GetArenaForAllocation());
    _impl_.state_ = p;
  }
  return _impl_.state_;
}
inline ::dtproto::robot_msgs::RobotState* RobotStateTimeStamped::mutable_state() {
  ::dtproto::robot_msgs::RobotState* _msg = _internal_mutable_state();
  // @@protoc_insertion_point(field_mutable:dtproto.robot_msgs.RobotStateTimeStamped.state)
  return _msg;
}
inline void RobotStateTimeStamped::set_allocated_state(::dtproto::robot_msgs::RobotState* state) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete _impl_.state_;
  }
  if (state) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalGetOwningArena(state);
    if (message_arena != submessage_arena) {
      state = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, state, submessage_arena);
    }
    
  } else {
    
  }
  _impl_.state_ = state;
  // @@protoc_insertion_point(field_set_allocated:dtproto.robot_msgs.RobotStateTimeStamped.state)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace robot_msgs
}  // namespace dtproto

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_dtProto_2frobot_5fmsgs_2fRobotState_2eproto
