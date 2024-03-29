// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: dtProto/geometry_msgs/Orientation.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_dtProto_2fgeometry_5fmsgs_2fOrientation_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_dtProto_2fgeometry_5fmsgs_2fOrientation_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_dtProto_2fgeometry_5fmsgs_2fOrientation_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_dtProto_2fgeometry_5fmsgs_2fOrientation_2eproto {
  static const uint32_t offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_dtProto_2fgeometry_5fmsgs_2fOrientation_2eproto;
namespace dtproto {
namespace geometry_msgs {
class Euler;
struct EulerDefaultTypeInternal;
extern EulerDefaultTypeInternal _Euler_default_instance_;
class Quaternion;
struct QuaternionDefaultTypeInternal;
extern QuaternionDefaultTypeInternal _Quaternion_default_instance_;
class Rotation;
struct RotationDefaultTypeInternal;
extern RotationDefaultTypeInternal _Rotation_default_instance_;
}  // namespace geometry_msgs
}  // namespace dtproto
PROTOBUF_NAMESPACE_OPEN
template<> ::dtproto::geometry_msgs::Euler* Arena::CreateMaybeMessage<::dtproto::geometry_msgs::Euler>(Arena*);
template<> ::dtproto::geometry_msgs::Quaternion* Arena::CreateMaybeMessage<::dtproto::geometry_msgs::Quaternion>(Arena*);
template<> ::dtproto::geometry_msgs::Rotation* Arena::CreateMaybeMessage<::dtproto::geometry_msgs::Rotation>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace dtproto {
namespace geometry_msgs {

// ===================================================================

class Euler final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:dtproto.geometry_msgs.Euler) */ {
 public:
  inline Euler() : Euler(nullptr) {}
  ~Euler() override;
  explicit PROTOBUF_CONSTEXPR Euler(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  Euler(const Euler& from);
  Euler(Euler&& from) noexcept
    : Euler() {
    *this = ::std::move(from);
  }

  inline Euler& operator=(const Euler& from) {
    CopyFrom(from);
    return *this;
  }
  inline Euler& operator=(Euler&& from) noexcept {
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
  static const Euler& default_instance() {
    return *internal_default_instance();
  }
  static inline const Euler* internal_default_instance() {
    return reinterpret_cast<const Euler*>(
               &_Euler_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(Euler& a, Euler& b) {
    a.Swap(&b);
  }
  inline void Swap(Euler* other) {
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
  void UnsafeArenaSwap(Euler* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  Euler* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<Euler>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const Euler& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom( const Euler& from) {
    Euler::MergeImpl(*this, from);
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
  void InternalSwap(Euler* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "dtproto.geometry_msgs.Euler";
  }
  protected:
  explicit Euler(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kRFieldNumber = 1,
    kPFieldNumber = 2,
    kYFieldNumber = 3,
  };
  // double r = 1;
  void clear_r();
  double r() const;
  void set_r(double value);
  private:
  double _internal_r() const;
  void _internal_set_r(double value);
  public:

  // double p = 2;
  void clear_p();
  double p() const;
  void set_p(double value);
  private:
  double _internal_p() const;
  void _internal_set_p(double value);
  public:

  // double y = 3;
  void clear_y();
  double y() const;
  void set_y(double value);
  private:
  double _internal_y() const;
  void _internal_set_y(double value);
  public:

  // @@protoc_insertion_point(class_scope:dtproto.geometry_msgs.Euler)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  struct Impl_ {
    double r_;
    double p_;
    double y_;
    mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  };
  union { Impl_ _impl_; };
  friend struct ::TableStruct_dtProto_2fgeometry_5fmsgs_2fOrientation_2eproto;
};
// -------------------------------------------------------------------

class Quaternion final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:dtproto.geometry_msgs.Quaternion) */ {
 public:
  inline Quaternion() : Quaternion(nullptr) {}
  ~Quaternion() override;
  explicit PROTOBUF_CONSTEXPR Quaternion(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  Quaternion(const Quaternion& from);
  Quaternion(Quaternion&& from) noexcept
    : Quaternion() {
    *this = ::std::move(from);
  }

  inline Quaternion& operator=(const Quaternion& from) {
    CopyFrom(from);
    return *this;
  }
  inline Quaternion& operator=(Quaternion&& from) noexcept {
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
  static const Quaternion& default_instance() {
    return *internal_default_instance();
  }
  static inline const Quaternion* internal_default_instance() {
    return reinterpret_cast<const Quaternion*>(
               &_Quaternion_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(Quaternion& a, Quaternion& b) {
    a.Swap(&b);
  }
  inline void Swap(Quaternion* other) {
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
  void UnsafeArenaSwap(Quaternion* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  Quaternion* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<Quaternion>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const Quaternion& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom( const Quaternion& from) {
    Quaternion::MergeImpl(*this, from);
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
  void InternalSwap(Quaternion* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "dtproto.geometry_msgs.Quaternion";
  }
  protected:
  explicit Quaternion(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kXFieldNumber = 1,
    kYFieldNumber = 2,
    kZFieldNumber = 3,
    kWFieldNumber = 4,
  };
  // double x = 1;
  void clear_x();
  double x() const;
  void set_x(double value);
  private:
  double _internal_x() const;
  void _internal_set_x(double value);
  public:

  // double y = 2;
  void clear_y();
  double y() const;
  void set_y(double value);
  private:
  double _internal_y() const;
  void _internal_set_y(double value);
  public:

  // double z = 3;
  void clear_z();
  double z() const;
  void set_z(double value);
  private:
  double _internal_z() const;
  void _internal_set_z(double value);
  public:

  // double w = 4;
  void clear_w();
  double w() const;
  void set_w(double value);
  private:
  double _internal_w() const;
  void _internal_set_w(double value);
  public:

  // @@protoc_insertion_point(class_scope:dtproto.geometry_msgs.Quaternion)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  struct Impl_ {
    double x_;
    double y_;
    double z_;
    double w_;
    mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  };
  union { Impl_ _impl_; };
  friend struct ::TableStruct_dtProto_2fgeometry_5fmsgs_2fOrientation_2eproto;
};
// -------------------------------------------------------------------

class Rotation final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:dtproto.geometry_msgs.Rotation) */ {
 public:
  inline Rotation() : Rotation(nullptr) {}
  ~Rotation() override;
  explicit PROTOBUF_CONSTEXPR Rotation(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  Rotation(const Rotation& from);
  Rotation(Rotation&& from) noexcept
    : Rotation() {
    *this = ::std::move(from);
  }

  inline Rotation& operator=(const Rotation& from) {
    CopyFrom(from);
    return *this;
  }
  inline Rotation& operator=(Rotation&& from) noexcept {
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
  static const Rotation& default_instance() {
    return *internal_default_instance();
  }
  static inline const Rotation* internal_default_instance() {
    return reinterpret_cast<const Rotation*>(
               &_Rotation_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    2;

  friend void swap(Rotation& a, Rotation& b) {
    a.Swap(&b);
  }
  inline void Swap(Rotation* other) {
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
  void UnsafeArenaSwap(Rotation* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  Rotation* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<Rotation>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const Rotation& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom( const Rotation& from) {
    Rotation::MergeImpl(*this, from);
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
  void InternalSwap(Rotation* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "dtproto.geometry_msgs.Rotation";
  }
  protected:
  explicit Rotation(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kA11FieldNumber = 1,
    kA12FieldNumber = 2,
    kA13FieldNumber = 3,
    kA21FieldNumber = 4,
    kA22FieldNumber = 5,
    kA23FieldNumber = 6,
    kA31FieldNumber = 7,
    kA32FieldNumber = 8,
    kA33FieldNumber = 9,
  };
  // double a11 = 1;
  void clear_a11();
  double a11() const;
  void set_a11(double value);
  private:
  double _internal_a11() const;
  void _internal_set_a11(double value);
  public:

  // double a12 = 2;
  void clear_a12();
  double a12() const;
  void set_a12(double value);
  private:
  double _internal_a12() const;
  void _internal_set_a12(double value);
  public:

  // double a13 = 3;
  void clear_a13();
  double a13() const;
  void set_a13(double value);
  private:
  double _internal_a13() const;
  void _internal_set_a13(double value);
  public:

  // double a21 = 4;
  void clear_a21();
  double a21() const;
  void set_a21(double value);
  private:
  double _internal_a21() const;
  void _internal_set_a21(double value);
  public:

  // double a22 = 5;
  void clear_a22();
  double a22() const;
  void set_a22(double value);
  private:
  double _internal_a22() const;
  void _internal_set_a22(double value);
  public:

  // double a23 = 6;
  void clear_a23();
  double a23() const;
  void set_a23(double value);
  private:
  double _internal_a23() const;
  void _internal_set_a23(double value);
  public:

  // double a31 = 7;
  void clear_a31();
  double a31() const;
  void set_a31(double value);
  private:
  double _internal_a31() const;
  void _internal_set_a31(double value);
  public:

  // double a32 = 8;
  void clear_a32();
  double a32() const;
  void set_a32(double value);
  private:
  double _internal_a32() const;
  void _internal_set_a32(double value);
  public:

  // double a33 = 9;
  void clear_a33();
  double a33() const;
  void set_a33(double value);
  private:
  double _internal_a33() const;
  void _internal_set_a33(double value);
  public:

  // @@protoc_insertion_point(class_scope:dtproto.geometry_msgs.Rotation)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  struct Impl_ {
    double a11_;
    double a12_;
    double a13_;
    double a21_;
    double a22_;
    double a23_;
    double a31_;
    double a32_;
    double a33_;
    mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  };
  union { Impl_ _impl_; };
  friend struct ::TableStruct_dtProto_2fgeometry_5fmsgs_2fOrientation_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Euler

// double r = 1;
inline void Euler::clear_r() {
  _impl_.r_ = 0;
}
inline double Euler::_internal_r() const {
  return _impl_.r_;
}
inline double Euler::r() const {
  // @@protoc_insertion_point(field_get:dtproto.geometry_msgs.Euler.r)
  return _internal_r();
}
inline void Euler::_internal_set_r(double value) {
  
  _impl_.r_ = value;
}
inline void Euler::set_r(double value) {
  _internal_set_r(value);
  // @@protoc_insertion_point(field_set:dtproto.geometry_msgs.Euler.r)
}

// double p = 2;
inline void Euler::clear_p() {
  _impl_.p_ = 0;
}
inline double Euler::_internal_p() const {
  return _impl_.p_;
}
inline double Euler::p() const {
  // @@protoc_insertion_point(field_get:dtproto.geometry_msgs.Euler.p)
  return _internal_p();
}
inline void Euler::_internal_set_p(double value) {
  
  _impl_.p_ = value;
}
inline void Euler::set_p(double value) {
  _internal_set_p(value);
  // @@protoc_insertion_point(field_set:dtproto.geometry_msgs.Euler.p)
}

// double y = 3;
inline void Euler::clear_y() {
  _impl_.y_ = 0;
}
inline double Euler::_internal_y() const {
  return _impl_.y_;
}
inline double Euler::y() const {
  // @@protoc_insertion_point(field_get:dtproto.geometry_msgs.Euler.y)
  return _internal_y();
}
inline void Euler::_internal_set_y(double value) {
  
  _impl_.y_ = value;
}
inline void Euler::set_y(double value) {
  _internal_set_y(value);
  // @@protoc_insertion_point(field_set:dtproto.geometry_msgs.Euler.y)
}

// -------------------------------------------------------------------

// Quaternion

// double x = 1;
inline void Quaternion::clear_x() {
  _impl_.x_ = 0;
}
inline double Quaternion::_internal_x() const {
  return _impl_.x_;
}
inline double Quaternion::x() const {
  // @@protoc_insertion_point(field_get:dtproto.geometry_msgs.Quaternion.x)
  return _internal_x();
}
inline void Quaternion::_internal_set_x(double value) {
  
  _impl_.x_ = value;
}
inline void Quaternion::set_x(double value) {
  _internal_set_x(value);
  // @@protoc_insertion_point(field_set:dtproto.geometry_msgs.Quaternion.x)
}

// double y = 2;
inline void Quaternion::clear_y() {
  _impl_.y_ = 0;
}
inline double Quaternion::_internal_y() const {
  return _impl_.y_;
}
inline double Quaternion::y() const {
  // @@protoc_insertion_point(field_get:dtproto.geometry_msgs.Quaternion.y)
  return _internal_y();
}
inline void Quaternion::_internal_set_y(double value) {
  
  _impl_.y_ = value;
}
inline void Quaternion::set_y(double value) {
  _internal_set_y(value);
  // @@protoc_insertion_point(field_set:dtproto.geometry_msgs.Quaternion.y)
}

// double z = 3;
inline void Quaternion::clear_z() {
  _impl_.z_ = 0;
}
inline double Quaternion::_internal_z() const {
  return _impl_.z_;
}
inline double Quaternion::z() const {
  // @@protoc_insertion_point(field_get:dtproto.geometry_msgs.Quaternion.z)
  return _internal_z();
}
inline void Quaternion::_internal_set_z(double value) {
  
  _impl_.z_ = value;
}
inline void Quaternion::set_z(double value) {
  _internal_set_z(value);
  // @@protoc_insertion_point(field_set:dtproto.geometry_msgs.Quaternion.z)
}

// double w = 4;
inline void Quaternion::clear_w() {
  _impl_.w_ = 0;
}
inline double Quaternion::_internal_w() const {
  return _impl_.w_;
}
inline double Quaternion::w() const {
  // @@protoc_insertion_point(field_get:dtproto.geometry_msgs.Quaternion.w)
  return _internal_w();
}
inline void Quaternion::_internal_set_w(double value) {
  
  _impl_.w_ = value;
}
inline void Quaternion::set_w(double value) {
  _internal_set_w(value);
  // @@protoc_insertion_point(field_set:dtproto.geometry_msgs.Quaternion.w)
}

// -------------------------------------------------------------------

// Rotation

// double a11 = 1;
inline void Rotation::clear_a11() {
  _impl_.a11_ = 0;
}
inline double Rotation::_internal_a11() const {
  return _impl_.a11_;
}
inline double Rotation::a11() const {
  // @@protoc_insertion_point(field_get:dtproto.geometry_msgs.Rotation.a11)
  return _internal_a11();
}
inline void Rotation::_internal_set_a11(double value) {
  
  _impl_.a11_ = value;
}
inline void Rotation::set_a11(double value) {
  _internal_set_a11(value);
  // @@protoc_insertion_point(field_set:dtproto.geometry_msgs.Rotation.a11)
}

// double a12 = 2;
inline void Rotation::clear_a12() {
  _impl_.a12_ = 0;
}
inline double Rotation::_internal_a12() const {
  return _impl_.a12_;
}
inline double Rotation::a12() const {
  // @@protoc_insertion_point(field_get:dtproto.geometry_msgs.Rotation.a12)
  return _internal_a12();
}
inline void Rotation::_internal_set_a12(double value) {
  
  _impl_.a12_ = value;
}
inline void Rotation::set_a12(double value) {
  _internal_set_a12(value);
  // @@protoc_insertion_point(field_set:dtproto.geometry_msgs.Rotation.a12)
}

// double a13 = 3;
inline void Rotation::clear_a13() {
  _impl_.a13_ = 0;
}
inline double Rotation::_internal_a13() const {
  return _impl_.a13_;
}
inline double Rotation::a13() const {
  // @@protoc_insertion_point(field_get:dtproto.geometry_msgs.Rotation.a13)
  return _internal_a13();
}
inline void Rotation::_internal_set_a13(double value) {
  
  _impl_.a13_ = value;
}
inline void Rotation::set_a13(double value) {
  _internal_set_a13(value);
  // @@protoc_insertion_point(field_set:dtproto.geometry_msgs.Rotation.a13)
}

// double a21 = 4;
inline void Rotation::clear_a21() {
  _impl_.a21_ = 0;
}
inline double Rotation::_internal_a21() const {
  return _impl_.a21_;
}
inline double Rotation::a21() const {
  // @@protoc_insertion_point(field_get:dtproto.geometry_msgs.Rotation.a21)
  return _internal_a21();
}
inline void Rotation::_internal_set_a21(double value) {
  
  _impl_.a21_ = value;
}
inline void Rotation::set_a21(double value) {
  _internal_set_a21(value);
  // @@protoc_insertion_point(field_set:dtproto.geometry_msgs.Rotation.a21)
}

// double a22 = 5;
inline void Rotation::clear_a22() {
  _impl_.a22_ = 0;
}
inline double Rotation::_internal_a22() const {
  return _impl_.a22_;
}
inline double Rotation::a22() const {
  // @@protoc_insertion_point(field_get:dtproto.geometry_msgs.Rotation.a22)
  return _internal_a22();
}
inline void Rotation::_internal_set_a22(double value) {
  
  _impl_.a22_ = value;
}
inline void Rotation::set_a22(double value) {
  _internal_set_a22(value);
  // @@protoc_insertion_point(field_set:dtproto.geometry_msgs.Rotation.a22)
}

// double a23 = 6;
inline void Rotation::clear_a23() {
  _impl_.a23_ = 0;
}
inline double Rotation::_internal_a23() const {
  return _impl_.a23_;
}
inline double Rotation::a23() const {
  // @@protoc_insertion_point(field_get:dtproto.geometry_msgs.Rotation.a23)
  return _internal_a23();
}
inline void Rotation::_internal_set_a23(double value) {
  
  _impl_.a23_ = value;
}
inline void Rotation::set_a23(double value) {
  _internal_set_a23(value);
  // @@protoc_insertion_point(field_set:dtproto.geometry_msgs.Rotation.a23)
}

// double a31 = 7;
inline void Rotation::clear_a31() {
  _impl_.a31_ = 0;
}
inline double Rotation::_internal_a31() const {
  return _impl_.a31_;
}
inline double Rotation::a31() const {
  // @@protoc_insertion_point(field_get:dtproto.geometry_msgs.Rotation.a31)
  return _internal_a31();
}
inline void Rotation::_internal_set_a31(double value) {
  
  _impl_.a31_ = value;
}
inline void Rotation::set_a31(double value) {
  _internal_set_a31(value);
  // @@protoc_insertion_point(field_set:dtproto.geometry_msgs.Rotation.a31)
}

// double a32 = 8;
inline void Rotation::clear_a32() {
  _impl_.a32_ = 0;
}
inline double Rotation::_internal_a32() const {
  return _impl_.a32_;
}
inline double Rotation::a32() const {
  // @@protoc_insertion_point(field_get:dtproto.geometry_msgs.Rotation.a32)
  return _internal_a32();
}
inline void Rotation::_internal_set_a32(double value) {
  
  _impl_.a32_ = value;
}
inline void Rotation::set_a32(double value) {
  _internal_set_a32(value);
  // @@protoc_insertion_point(field_set:dtproto.geometry_msgs.Rotation.a32)
}

// double a33 = 9;
inline void Rotation::clear_a33() {
  _impl_.a33_ = 0;
}
inline double Rotation::_internal_a33() const {
  return _impl_.a33_;
}
inline double Rotation::a33() const {
  // @@protoc_insertion_point(field_get:dtproto.geometry_msgs.Rotation.a33)
  return _internal_a33();
}
inline void Rotation::_internal_set_a33(double value) {
  
  _impl_.a33_ = value;
}
inline void Rotation::set_a33(double value) {
  _internal_set_a33(value);
  // @@protoc_insertion_point(field_set:dtproto.geometry_msgs.Rotation.a33)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------

// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace geometry_msgs
}  // namespace dtproto

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_dtProto_2fgeometry_5fmsgs_2fOrientation_2eproto
