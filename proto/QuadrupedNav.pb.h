// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: QuadrupedNav.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_QuadrupedNav_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_QuadrupedNav_2eproto

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
#include "dtProto/nav_msgs/Odom.pb.h"
#include "dtProto/nav_msgs/Grid.pb.h"
#include "dtProto/nav_msgs/SteppableArea.pb.h"
#include "dtProto/std_msgs/Request.pb.h"
#include "dtProto/std_msgs/Header.pb.h"
#include "dtProto/geometry_msgs/Point.pb.h"
#include "dtProto/geometry_msgs/Vector.pb.h"
#include "dtProto/sensor_msgs/Imu.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_QuadrupedNav_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_QuadrupedNav_2eproto {
  static const uint32_t offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_QuadrupedNav_2eproto;
namespace dtproto {
namespace quadruped {
class OdomWithJointPosTimeStamped;
struct OdomWithJointPosTimeStampedDefaultTypeInternal;
extern OdomWithJointPosTimeStampedDefaultTypeInternal _OdomWithJointPosTimeStamped_default_instance_;
}  // namespace quadruped
}  // namespace dtproto
PROTOBUF_NAMESPACE_OPEN
template<> ::dtproto::quadruped::OdomWithJointPosTimeStamped* Arena::CreateMaybeMessage<::dtproto::quadruped::OdomWithJointPosTimeStamped>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace dtproto {
namespace quadruped {

// ===================================================================

class OdomWithJointPosTimeStamped final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:dtproto.quadruped.OdomWithJointPosTimeStamped) */ {
 public:
  inline OdomWithJointPosTimeStamped() : OdomWithJointPosTimeStamped(nullptr) {}
  ~OdomWithJointPosTimeStamped() override;
  explicit PROTOBUF_CONSTEXPR OdomWithJointPosTimeStamped(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  OdomWithJointPosTimeStamped(const OdomWithJointPosTimeStamped& from);
  OdomWithJointPosTimeStamped(OdomWithJointPosTimeStamped&& from) noexcept
    : OdomWithJointPosTimeStamped() {
    *this = ::std::move(from);
  }

  inline OdomWithJointPosTimeStamped& operator=(const OdomWithJointPosTimeStamped& from) {
    CopyFrom(from);
    return *this;
  }
  inline OdomWithJointPosTimeStamped& operator=(OdomWithJointPosTimeStamped&& from) noexcept {
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
  static const OdomWithJointPosTimeStamped& default_instance() {
    return *internal_default_instance();
  }
  static inline const OdomWithJointPosTimeStamped* internal_default_instance() {
    return reinterpret_cast<const OdomWithJointPosTimeStamped*>(
               &_OdomWithJointPosTimeStamped_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(OdomWithJointPosTimeStamped& a, OdomWithJointPosTimeStamped& b) {
    a.Swap(&b);
  }
  inline void Swap(OdomWithJointPosTimeStamped* other) {
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
  void UnsafeArenaSwap(OdomWithJointPosTimeStamped* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  OdomWithJointPosTimeStamped* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<OdomWithJointPosTimeStamped>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const OdomWithJointPosTimeStamped& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom( const OdomWithJointPosTimeStamped& from) {
    OdomWithJointPosTimeStamped::MergeImpl(*this, from);
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
  void InternalSwap(OdomWithJointPosTimeStamped* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "dtproto.quadruped.OdomWithJointPosTimeStamped";
  }
  protected:
  explicit OdomWithJointPosTimeStamped(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kJointPosFieldNumber = 3,
    kFootPosFieldNumber = 4,
    kHeaderFieldNumber = 1,
    kOdomFieldNumber = 2,
    kContactFieldNumber = 5,
  };
  // repeated double joint_pos = 3;
  int joint_pos_size() const;
  private:
  int _internal_joint_pos_size() const;
  public:
  void clear_joint_pos();
  private:
  double _internal_joint_pos(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      _internal_joint_pos() const;
  void _internal_add_joint_pos(double value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      _internal_mutable_joint_pos();
  public:
  double joint_pos(int index) const;
  void set_joint_pos(int index, double value);
  void add_joint_pos(double value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      joint_pos() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      mutable_joint_pos();

  // repeated .dtproto.geometry_msgs.Point3d foot_pos = 4;
  int foot_pos_size() const;
  private:
  int _internal_foot_pos_size() const;
  public:
  void clear_foot_pos();
  ::dtproto::geometry_msgs::Point3d* mutable_foot_pos(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::dtproto::geometry_msgs::Point3d >*
      mutable_foot_pos();
  private:
  const ::dtproto::geometry_msgs::Point3d& _internal_foot_pos(int index) const;
  ::dtproto::geometry_msgs::Point3d* _internal_add_foot_pos();
  public:
  const ::dtproto::geometry_msgs::Point3d& foot_pos(int index) const;
  ::dtproto::geometry_msgs::Point3d* add_foot_pos();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::dtproto::geometry_msgs::Point3d >&
      foot_pos() const;

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

  // .dtproto.nav_msgs.Odom odom = 2;
  bool has_odom() const;
  private:
  bool _internal_has_odom() const;
  public:
  void clear_odom();
  const ::dtproto::nav_msgs::Odom& odom() const;
  PROTOBUF_NODISCARD ::dtproto::nav_msgs::Odom* release_odom();
  ::dtproto::nav_msgs::Odom* mutable_odom();
  void set_allocated_odom(::dtproto::nav_msgs::Odom* odom);
  private:
  const ::dtproto::nav_msgs::Odom& _internal_odom() const;
  ::dtproto::nav_msgs::Odom* _internal_mutable_odom();
  public:
  void unsafe_arena_set_allocated_odom(
      ::dtproto::nav_msgs::Odom* odom);
  ::dtproto::nav_msgs::Odom* unsafe_arena_release_odom();

  // .dtproto.geometry_msgs.Vector4b contact = 5;
  bool has_contact() const;
  private:
  bool _internal_has_contact() const;
  public:
  void clear_contact();
  const ::dtproto::geometry_msgs::Vector4b& contact() const;
  PROTOBUF_NODISCARD ::dtproto::geometry_msgs::Vector4b* release_contact();
  ::dtproto::geometry_msgs::Vector4b* mutable_contact();
  void set_allocated_contact(::dtproto::geometry_msgs::Vector4b* contact);
  private:
  const ::dtproto::geometry_msgs::Vector4b& _internal_contact() const;
  ::dtproto::geometry_msgs::Vector4b* _internal_mutable_contact();
  public:
  void unsafe_arena_set_allocated_contact(
      ::dtproto::geometry_msgs::Vector4b* contact);
  ::dtproto::geometry_msgs::Vector4b* unsafe_arena_release_contact();

  // @@protoc_insertion_point(class_scope:dtproto.quadruped.OdomWithJointPosTimeStamped)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  struct Impl_ {
    ::PROTOBUF_NAMESPACE_ID::RepeatedField< double > joint_pos_;
    ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::dtproto::geometry_msgs::Point3d > foot_pos_;
    ::dtproto::std_msgs::Header* header_;
    ::dtproto::nav_msgs::Odom* odom_;
    ::dtproto::geometry_msgs::Vector4b* contact_;
    mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  };
  union { Impl_ _impl_; };
  friend struct ::TableStruct_QuadrupedNav_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// OdomWithJointPosTimeStamped

// .dtproto.std_msgs.Header header = 1;
inline bool OdomWithJointPosTimeStamped::_internal_has_header() const {
  return this != internal_default_instance() && _impl_.header_ != nullptr;
}
inline bool OdomWithJointPosTimeStamped::has_header() const {
  return _internal_has_header();
}
inline const ::dtproto::std_msgs::Header& OdomWithJointPosTimeStamped::_internal_header() const {
  const ::dtproto::std_msgs::Header* p = _impl_.header_;
  return p != nullptr ? *p : reinterpret_cast<const ::dtproto::std_msgs::Header&>(
      ::dtproto::std_msgs::_Header_default_instance_);
}
inline const ::dtproto::std_msgs::Header& OdomWithJointPosTimeStamped::header() const {
  // @@protoc_insertion_point(field_get:dtproto.quadruped.OdomWithJointPosTimeStamped.header)
  return _internal_header();
}
inline void OdomWithJointPosTimeStamped::unsafe_arena_set_allocated_header(
    ::dtproto::std_msgs::Header* header) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.header_);
  }
  _impl_.header_ = header;
  if (header) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:dtproto.quadruped.OdomWithJointPosTimeStamped.header)
}
inline ::dtproto::std_msgs::Header* OdomWithJointPosTimeStamped::release_header() {
  
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
inline ::dtproto::std_msgs::Header* OdomWithJointPosTimeStamped::unsafe_arena_release_header() {
  // @@protoc_insertion_point(field_release:dtproto.quadruped.OdomWithJointPosTimeStamped.header)
  
  ::dtproto::std_msgs::Header* temp = _impl_.header_;
  _impl_.header_ = nullptr;
  return temp;
}
inline ::dtproto::std_msgs::Header* OdomWithJointPosTimeStamped::_internal_mutable_header() {
  
  if (_impl_.header_ == nullptr) {
    auto* p = CreateMaybeMessage<::dtproto::std_msgs::Header>(GetArenaForAllocation());
    _impl_.header_ = p;
  }
  return _impl_.header_;
}
inline ::dtproto::std_msgs::Header* OdomWithJointPosTimeStamped::mutable_header() {
  ::dtproto::std_msgs::Header* _msg = _internal_mutable_header();
  // @@protoc_insertion_point(field_mutable:dtproto.quadruped.OdomWithJointPosTimeStamped.header)
  return _msg;
}
inline void OdomWithJointPosTimeStamped::set_allocated_header(::dtproto::std_msgs::Header* header) {
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
  // @@protoc_insertion_point(field_set_allocated:dtproto.quadruped.OdomWithJointPosTimeStamped.header)
}

// .dtproto.nav_msgs.Odom odom = 2;
inline bool OdomWithJointPosTimeStamped::_internal_has_odom() const {
  return this != internal_default_instance() && _impl_.odom_ != nullptr;
}
inline bool OdomWithJointPosTimeStamped::has_odom() const {
  return _internal_has_odom();
}
inline const ::dtproto::nav_msgs::Odom& OdomWithJointPosTimeStamped::_internal_odom() const {
  const ::dtproto::nav_msgs::Odom* p = _impl_.odom_;
  return p != nullptr ? *p : reinterpret_cast<const ::dtproto::nav_msgs::Odom&>(
      ::dtproto::nav_msgs::_Odom_default_instance_);
}
inline const ::dtproto::nav_msgs::Odom& OdomWithJointPosTimeStamped::odom() const {
  // @@protoc_insertion_point(field_get:dtproto.quadruped.OdomWithJointPosTimeStamped.odom)
  return _internal_odom();
}
inline void OdomWithJointPosTimeStamped::unsafe_arena_set_allocated_odom(
    ::dtproto::nav_msgs::Odom* odom) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.odom_);
  }
  _impl_.odom_ = odom;
  if (odom) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:dtproto.quadruped.OdomWithJointPosTimeStamped.odom)
}
inline ::dtproto::nav_msgs::Odom* OdomWithJointPosTimeStamped::release_odom() {
  
  ::dtproto::nav_msgs::Odom* temp = _impl_.odom_;
  _impl_.odom_ = nullptr;
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
inline ::dtproto::nav_msgs::Odom* OdomWithJointPosTimeStamped::unsafe_arena_release_odom() {
  // @@protoc_insertion_point(field_release:dtproto.quadruped.OdomWithJointPosTimeStamped.odom)
  
  ::dtproto::nav_msgs::Odom* temp = _impl_.odom_;
  _impl_.odom_ = nullptr;
  return temp;
}
inline ::dtproto::nav_msgs::Odom* OdomWithJointPosTimeStamped::_internal_mutable_odom() {
  
  if (_impl_.odom_ == nullptr) {
    auto* p = CreateMaybeMessage<::dtproto::nav_msgs::Odom>(GetArenaForAllocation());
    _impl_.odom_ = p;
  }
  return _impl_.odom_;
}
inline ::dtproto::nav_msgs::Odom* OdomWithJointPosTimeStamped::mutable_odom() {
  ::dtproto::nav_msgs::Odom* _msg = _internal_mutable_odom();
  // @@protoc_insertion_point(field_mutable:dtproto.quadruped.OdomWithJointPosTimeStamped.odom)
  return _msg;
}
inline void OdomWithJointPosTimeStamped::set_allocated_odom(::dtproto::nav_msgs::Odom* odom) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.odom_);
  }
  if (odom) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalGetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(odom));
    if (message_arena != submessage_arena) {
      odom = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, odom, submessage_arena);
    }
    
  } else {
    
  }
  _impl_.odom_ = odom;
  // @@protoc_insertion_point(field_set_allocated:dtproto.quadruped.OdomWithJointPosTimeStamped.odom)
}

// repeated double joint_pos = 3;
inline int OdomWithJointPosTimeStamped::_internal_joint_pos_size() const {
  return _impl_.joint_pos_.size();
}
inline int OdomWithJointPosTimeStamped::joint_pos_size() const {
  return _internal_joint_pos_size();
}
inline void OdomWithJointPosTimeStamped::clear_joint_pos() {
  _impl_.joint_pos_.Clear();
}
inline double OdomWithJointPosTimeStamped::_internal_joint_pos(int index) const {
  return _impl_.joint_pos_.Get(index);
}
inline double OdomWithJointPosTimeStamped::joint_pos(int index) const {
  // @@protoc_insertion_point(field_get:dtproto.quadruped.OdomWithJointPosTimeStamped.joint_pos)
  return _internal_joint_pos(index);
}
inline void OdomWithJointPosTimeStamped::set_joint_pos(int index, double value) {
  _impl_.joint_pos_.Set(index, value);
  // @@protoc_insertion_point(field_set:dtproto.quadruped.OdomWithJointPosTimeStamped.joint_pos)
}
inline void OdomWithJointPosTimeStamped::_internal_add_joint_pos(double value) {
  _impl_.joint_pos_.Add(value);
}
inline void OdomWithJointPosTimeStamped::add_joint_pos(double value) {
  _internal_add_joint_pos(value);
  // @@protoc_insertion_point(field_add:dtproto.quadruped.OdomWithJointPosTimeStamped.joint_pos)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
OdomWithJointPosTimeStamped::_internal_joint_pos() const {
  return _impl_.joint_pos_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
OdomWithJointPosTimeStamped::joint_pos() const {
  // @@protoc_insertion_point(field_list:dtproto.quadruped.OdomWithJointPosTimeStamped.joint_pos)
  return _internal_joint_pos();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
OdomWithJointPosTimeStamped::_internal_mutable_joint_pos() {
  return &_impl_.joint_pos_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
OdomWithJointPosTimeStamped::mutable_joint_pos() {
  // @@protoc_insertion_point(field_mutable_list:dtproto.quadruped.OdomWithJointPosTimeStamped.joint_pos)
  return _internal_mutable_joint_pos();
}

// repeated .dtproto.geometry_msgs.Point3d foot_pos = 4;
inline int OdomWithJointPosTimeStamped::_internal_foot_pos_size() const {
  return _impl_.foot_pos_.size();
}
inline int OdomWithJointPosTimeStamped::foot_pos_size() const {
  return _internal_foot_pos_size();
}
inline ::dtproto::geometry_msgs::Point3d* OdomWithJointPosTimeStamped::mutable_foot_pos(int index) {
  // @@protoc_insertion_point(field_mutable:dtproto.quadruped.OdomWithJointPosTimeStamped.foot_pos)
  return _impl_.foot_pos_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::dtproto::geometry_msgs::Point3d >*
OdomWithJointPosTimeStamped::mutable_foot_pos() {
  // @@protoc_insertion_point(field_mutable_list:dtproto.quadruped.OdomWithJointPosTimeStamped.foot_pos)
  return &_impl_.foot_pos_;
}
inline const ::dtproto::geometry_msgs::Point3d& OdomWithJointPosTimeStamped::_internal_foot_pos(int index) const {
  return _impl_.foot_pos_.Get(index);
}
inline const ::dtproto::geometry_msgs::Point3d& OdomWithJointPosTimeStamped::foot_pos(int index) const {
  // @@protoc_insertion_point(field_get:dtproto.quadruped.OdomWithJointPosTimeStamped.foot_pos)
  return _internal_foot_pos(index);
}
inline ::dtproto::geometry_msgs::Point3d* OdomWithJointPosTimeStamped::_internal_add_foot_pos() {
  return _impl_.foot_pos_.Add();
}
inline ::dtproto::geometry_msgs::Point3d* OdomWithJointPosTimeStamped::add_foot_pos() {
  ::dtproto::geometry_msgs::Point3d* _add = _internal_add_foot_pos();
  // @@protoc_insertion_point(field_add:dtproto.quadruped.OdomWithJointPosTimeStamped.foot_pos)
  return _add;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::dtproto::geometry_msgs::Point3d >&
OdomWithJointPosTimeStamped::foot_pos() const {
  // @@protoc_insertion_point(field_list:dtproto.quadruped.OdomWithJointPosTimeStamped.foot_pos)
  return _impl_.foot_pos_;
}

// .dtproto.geometry_msgs.Vector4b contact = 5;
inline bool OdomWithJointPosTimeStamped::_internal_has_contact() const {
  return this != internal_default_instance() && _impl_.contact_ != nullptr;
}
inline bool OdomWithJointPosTimeStamped::has_contact() const {
  return _internal_has_contact();
}
inline const ::dtproto::geometry_msgs::Vector4b& OdomWithJointPosTimeStamped::_internal_contact() const {
  const ::dtproto::geometry_msgs::Vector4b* p = _impl_.contact_;
  return p != nullptr ? *p : reinterpret_cast<const ::dtproto::geometry_msgs::Vector4b&>(
      ::dtproto::geometry_msgs::_Vector4b_default_instance_);
}
inline const ::dtproto::geometry_msgs::Vector4b& OdomWithJointPosTimeStamped::contact() const {
  // @@protoc_insertion_point(field_get:dtproto.quadruped.OdomWithJointPosTimeStamped.contact)
  return _internal_contact();
}
inline void OdomWithJointPosTimeStamped::unsafe_arena_set_allocated_contact(
    ::dtproto::geometry_msgs::Vector4b* contact) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.contact_);
  }
  _impl_.contact_ = contact;
  if (contact) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:dtproto.quadruped.OdomWithJointPosTimeStamped.contact)
}
inline ::dtproto::geometry_msgs::Vector4b* OdomWithJointPosTimeStamped::release_contact() {
  
  ::dtproto::geometry_msgs::Vector4b* temp = _impl_.contact_;
  _impl_.contact_ = nullptr;
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
inline ::dtproto::geometry_msgs::Vector4b* OdomWithJointPosTimeStamped::unsafe_arena_release_contact() {
  // @@protoc_insertion_point(field_release:dtproto.quadruped.OdomWithJointPosTimeStamped.contact)
  
  ::dtproto::geometry_msgs::Vector4b* temp = _impl_.contact_;
  _impl_.contact_ = nullptr;
  return temp;
}
inline ::dtproto::geometry_msgs::Vector4b* OdomWithJointPosTimeStamped::_internal_mutable_contact() {
  
  if (_impl_.contact_ == nullptr) {
    auto* p = CreateMaybeMessage<::dtproto::geometry_msgs::Vector4b>(GetArenaForAllocation());
    _impl_.contact_ = p;
  }
  return _impl_.contact_;
}
inline ::dtproto::geometry_msgs::Vector4b* OdomWithJointPosTimeStamped::mutable_contact() {
  ::dtproto::geometry_msgs::Vector4b* _msg = _internal_mutable_contact();
  // @@protoc_insertion_point(field_mutable:dtproto.quadruped.OdomWithJointPosTimeStamped.contact)
  return _msg;
}
inline void OdomWithJointPosTimeStamped::set_allocated_contact(::dtproto::geometry_msgs::Vector4b* contact) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.contact_);
  }
  if (contact) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalGetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(contact));
    if (message_arena != submessage_arena) {
      contact = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, contact, submessage_arena);
    }
    
  } else {
    
  }
  _impl_.contact_ = contact;
  // @@protoc_insertion_point(field_set_allocated:dtproto.quadruped.OdomWithJointPosTimeStamped.contact)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace quadruped
}  // namespace dtproto

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_QuadrupedNav_2eproto
