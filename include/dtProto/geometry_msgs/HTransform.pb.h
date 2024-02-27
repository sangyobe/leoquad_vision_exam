// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: dtProto/geometry_msgs/HTransform.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_dtProto_2fgeometry_5fmsgs_2fHTransform_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_dtProto_2fgeometry_5fmsgs_2fHTransform_2eproto

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
#include "dtProto/geometry_msgs/Displacement.pb.h"
#include "dtProto/geometry_msgs/Orientation.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_dtProto_2fgeometry_5fmsgs_2fHTransform_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_dtProto_2fgeometry_5fmsgs_2fHTransform_2eproto {
  static const uint32_t offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_dtProto_2fgeometry_5fmsgs_2fHTransform_2eproto;
namespace dtproto {
namespace geometry_msgs {
class HTransform;
struct HTransformDefaultTypeInternal;
extern HTransformDefaultTypeInternal _HTransform_default_instance_;
}  // namespace geometry_msgs
}  // namespace dtproto
PROTOBUF_NAMESPACE_OPEN
template<> ::dtproto::geometry_msgs::HTransform* Arena::CreateMaybeMessage<::dtproto::geometry_msgs::HTransform>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace dtproto {
namespace geometry_msgs {

// ===================================================================

class HTransform final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:dtproto.geometry_msgs.HTransform) */ {
 public:
  inline HTransform() : HTransform(nullptr) {}
  ~HTransform() override;
  explicit PROTOBUF_CONSTEXPR HTransform(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  HTransform(const HTransform& from);
  HTransform(HTransform&& from) noexcept
    : HTransform() {
    *this = ::std::move(from);
  }

  inline HTransform& operator=(const HTransform& from) {
    CopyFrom(from);
    return *this;
  }
  inline HTransform& operator=(HTransform&& from) noexcept {
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
  static const HTransform& default_instance() {
    return *internal_default_instance();
  }
  static inline const HTransform* internal_default_instance() {
    return reinterpret_cast<const HTransform*>(
               &_HTransform_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(HTransform& a, HTransform& b) {
    a.Swap(&b);
  }
  inline void Swap(HTransform* other) {
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
  void UnsafeArenaSwap(HTransform* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  HTransform* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<HTransform>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const HTransform& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom( const HTransform& from) {
    HTransform::MergeImpl(*this, from);
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
  void InternalSwap(HTransform* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "dtproto.geometry_msgs.HTransform";
  }
  protected:
  explicit HTransform(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kRotationFieldNumber = 1,
    kDisplacementFieldNumber = 2,
  };
  // .dtproto.geometry_msgs.Rotation rotation = 1;
  bool has_rotation() const;
  private:
  bool _internal_has_rotation() const;
  public:
  void clear_rotation();
  const ::dtproto::geometry_msgs::Rotation& rotation() const;
  PROTOBUF_NODISCARD ::dtproto::geometry_msgs::Rotation* release_rotation();
  ::dtproto::geometry_msgs::Rotation* mutable_rotation();
  void set_allocated_rotation(::dtproto::geometry_msgs::Rotation* rotation);
  private:
  const ::dtproto::geometry_msgs::Rotation& _internal_rotation() const;
  ::dtproto::geometry_msgs::Rotation* _internal_mutable_rotation();
  public:
  void unsafe_arena_set_allocated_rotation(
      ::dtproto::geometry_msgs::Rotation* rotation);
  ::dtproto::geometry_msgs::Rotation* unsafe_arena_release_rotation();

  // .dtproto.geometry_msgs.Displacement displacement = 2;
  bool has_displacement() const;
  private:
  bool _internal_has_displacement() const;
  public:
  void clear_displacement();
  const ::dtproto::geometry_msgs::Displacement& displacement() const;
  PROTOBUF_NODISCARD ::dtproto::geometry_msgs::Displacement* release_displacement();
  ::dtproto::geometry_msgs::Displacement* mutable_displacement();
  void set_allocated_displacement(::dtproto::geometry_msgs::Displacement* displacement);
  private:
  const ::dtproto::geometry_msgs::Displacement& _internal_displacement() const;
  ::dtproto::geometry_msgs::Displacement* _internal_mutable_displacement();
  public:
  void unsafe_arena_set_allocated_displacement(
      ::dtproto::geometry_msgs::Displacement* displacement);
  ::dtproto::geometry_msgs::Displacement* unsafe_arena_release_displacement();

  // @@protoc_insertion_point(class_scope:dtproto.geometry_msgs.HTransform)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  struct Impl_ {
    ::dtproto::geometry_msgs::Rotation* rotation_;
    ::dtproto::geometry_msgs::Displacement* displacement_;
    mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  };
  union { Impl_ _impl_; };
  friend struct ::TableStruct_dtProto_2fgeometry_5fmsgs_2fHTransform_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// HTransform

// .dtproto.geometry_msgs.Rotation rotation = 1;
inline bool HTransform::_internal_has_rotation() const {
  return this != internal_default_instance() && _impl_.rotation_ != nullptr;
}
inline bool HTransform::has_rotation() const {
  return _internal_has_rotation();
}
inline const ::dtproto::geometry_msgs::Rotation& HTransform::_internal_rotation() const {
  const ::dtproto::geometry_msgs::Rotation* p = _impl_.rotation_;
  return p != nullptr ? *p : reinterpret_cast<const ::dtproto::geometry_msgs::Rotation&>(
      ::dtproto::geometry_msgs::_Rotation_default_instance_);
}
inline const ::dtproto::geometry_msgs::Rotation& HTransform::rotation() const {
  // @@protoc_insertion_point(field_get:dtproto.geometry_msgs.HTransform.rotation)
  return _internal_rotation();
}
inline void HTransform::unsafe_arena_set_allocated_rotation(
    ::dtproto::geometry_msgs::Rotation* rotation) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.rotation_);
  }
  _impl_.rotation_ = rotation;
  if (rotation) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:dtproto.geometry_msgs.HTransform.rotation)
}
inline ::dtproto::geometry_msgs::Rotation* HTransform::release_rotation() {
  
  ::dtproto::geometry_msgs::Rotation* temp = _impl_.rotation_;
  _impl_.rotation_ = nullptr;
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
inline ::dtproto::geometry_msgs::Rotation* HTransform::unsafe_arena_release_rotation() {
  // @@protoc_insertion_point(field_release:dtproto.geometry_msgs.HTransform.rotation)
  
  ::dtproto::geometry_msgs::Rotation* temp = _impl_.rotation_;
  _impl_.rotation_ = nullptr;
  return temp;
}
inline ::dtproto::geometry_msgs::Rotation* HTransform::_internal_mutable_rotation() {
  
  if (_impl_.rotation_ == nullptr) {
    auto* p = CreateMaybeMessage<::dtproto::geometry_msgs::Rotation>(GetArenaForAllocation());
    _impl_.rotation_ = p;
  }
  return _impl_.rotation_;
}
inline ::dtproto::geometry_msgs::Rotation* HTransform::mutable_rotation() {
  ::dtproto::geometry_msgs::Rotation* _msg = _internal_mutable_rotation();
  // @@protoc_insertion_point(field_mutable:dtproto.geometry_msgs.HTransform.rotation)
  return _msg;
}
inline void HTransform::set_allocated_rotation(::dtproto::geometry_msgs::Rotation* rotation) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.rotation_);
  }
  if (rotation) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalGetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(rotation));
    if (message_arena != submessage_arena) {
      rotation = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, rotation, submessage_arena);
    }
    
  } else {
    
  }
  _impl_.rotation_ = rotation;
  // @@protoc_insertion_point(field_set_allocated:dtproto.geometry_msgs.HTransform.rotation)
}

// .dtproto.geometry_msgs.Displacement displacement = 2;
inline bool HTransform::_internal_has_displacement() const {
  return this != internal_default_instance() && _impl_.displacement_ != nullptr;
}
inline bool HTransform::has_displacement() const {
  return _internal_has_displacement();
}
inline const ::dtproto::geometry_msgs::Displacement& HTransform::_internal_displacement() const {
  const ::dtproto::geometry_msgs::Displacement* p = _impl_.displacement_;
  return p != nullptr ? *p : reinterpret_cast<const ::dtproto::geometry_msgs::Displacement&>(
      ::dtproto::geometry_msgs::_Displacement_default_instance_);
}
inline const ::dtproto::geometry_msgs::Displacement& HTransform::displacement() const {
  // @@protoc_insertion_point(field_get:dtproto.geometry_msgs.HTransform.displacement)
  return _internal_displacement();
}
inline void HTransform::unsafe_arena_set_allocated_displacement(
    ::dtproto::geometry_msgs::Displacement* displacement) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.displacement_);
  }
  _impl_.displacement_ = displacement;
  if (displacement) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:dtproto.geometry_msgs.HTransform.displacement)
}
inline ::dtproto::geometry_msgs::Displacement* HTransform::release_displacement() {
  
  ::dtproto::geometry_msgs::Displacement* temp = _impl_.displacement_;
  _impl_.displacement_ = nullptr;
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
inline ::dtproto::geometry_msgs::Displacement* HTransform::unsafe_arena_release_displacement() {
  // @@protoc_insertion_point(field_release:dtproto.geometry_msgs.HTransform.displacement)
  
  ::dtproto::geometry_msgs::Displacement* temp = _impl_.displacement_;
  _impl_.displacement_ = nullptr;
  return temp;
}
inline ::dtproto::geometry_msgs::Displacement* HTransform::_internal_mutable_displacement() {
  
  if (_impl_.displacement_ == nullptr) {
    auto* p = CreateMaybeMessage<::dtproto::geometry_msgs::Displacement>(GetArenaForAllocation());
    _impl_.displacement_ = p;
  }
  return _impl_.displacement_;
}
inline ::dtproto::geometry_msgs::Displacement* HTransform::mutable_displacement() {
  ::dtproto::geometry_msgs::Displacement* _msg = _internal_mutable_displacement();
  // @@protoc_insertion_point(field_mutable:dtproto.geometry_msgs.HTransform.displacement)
  return _msg;
}
inline void HTransform::set_allocated_displacement(::dtproto::geometry_msgs::Displacement* displacement) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.displacement_);
  }
  if (displacement) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalGetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(displacement));
    if (message_arena != submessage_arena) {
      displacement = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, displacement, submessage_arena);
    }
    
  } else {
    
  }
  _impl_.displacement_ = displacement;
  // @@protoc_insertion_point(field_set_allocated:dtproto.geometry_msgs.HTransform.displacement)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace geometry_msgs
}  // namespace dtproto

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_dtProto_2fgeometry_5fmsgs_2fHTransform_2eproto
