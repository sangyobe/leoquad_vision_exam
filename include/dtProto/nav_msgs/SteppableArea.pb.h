// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: dtProto/nav_msgs/SteppableArea.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_dtProto_2fnav_5fmsgs_2fSteppableArea_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_dtProto_2fnav_5fmsgs_2fSteppableArea_2eproto

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
#include "dtProto/geometry_msgs/Polygon.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_dtProto_2fnav_5fmsgs_2fSteppableArea_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_dtProto_2fnav_5fmsgs_2fSteppableArea_2eproto {
  static const uint32_t offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_dtProto_2fnav_5fmsgs_2fSteppableArea_2eproto;
namespace dtproto {
namespace nav_msgs {
class SteppableArea;
struct SteppableAreaDefaultTypeInternal;
extern SteppableAreaDefaultTypeInternal _SteppableArea_default_instance_;
class SteppableAreaTimeStamped;
struct SteppableAreaTimeStampedDefaultTypeInternal;
extern SteppableAreaTimeStampedDefaultTypeInternal _SteppableAreaTimeStamped_default_instance_;
}  // namespace nav_msgs
}  // namespace dtproto
PROTOBUF_NAMESPACE_OPEN
template<> ::dtproto::nav_msgs::SteppableArea* Arena::CreateMaybeMessage<::dtproto::nav_msgs::SteppableArea>(Arena*);
template<> ::dtproto::nav_msgs::SteppableAreaTimeStamped* Arena::CreateMaybeMessage<::dtproto::nav_msgs::SteppableAreaTimeStamped>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace dtproto {
namespace nav_msgs {

// ===================================================================

class SteppableArea final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:dtproto.nav_msgs.SteppableArea) */ {
 public:
  inline SteppableArea() : SteppableArea(nullptr) {}
  ~SteppableArea() override;
  explicit PROTOBUF_CONSTEXPR SteppableArea(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  SteppableArea(const SteppableArea& from);
  SteppableArea(SteppableArea&& from) noexcept
    : SteppableArea() {
    *this = ::std::move(from);
  }

  inline SteppableArea& operator=(const SteppableArea& from) {
    CopyFrom(from);
    return *this;
  }
  inline SteppableArea& operator=(SteppableArea&& from) noexcept {
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
  static const SteppableArea& default_instance() {
    return *internal_default_instance();
  }
  static inline const SteppableArea* internal_default_instance() {
    return reinterpret_cast<const SteppableArea*>(
               &_SteppableArea_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(SteppableArea& a, SteppableArea& b) {
    a.Swap(&b);
  }
  inline void Swap(SteppableArea* other) {
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
  void UnsafeArenaSwap(SteppableArea* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  SteppableArea* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<SteppableArea>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const SteppableArea& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom( const SteppableArea& from) {
    SteppableArea::MergeImpl(*this, from);
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
  void InternalSwap(SteppableArea* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "dtproto.nav_msgs.SteppableArea";
  }
  protected:
  explicit SteppableArea(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kSteppablesFieldNumber = 1,
    kUnsteppablesFieldNumber = 3,
    kSteppablesCountFieldNumber = 2,
    kUnsteppablesCountFieldNumber = 4,
  };
  // repeated .dtproto.geometry_msgs.Polygon3d steppables = 1;
  int steppables_size() const;
  private:
  int _internal_steppables_size() const;
  public:
  void clear_steppables();
  ::dtproto::geometry_msgs::Polygon3d* mutable_steppables(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::dtproto::geometry_msgs::Polygon3d >*
      mutable_steppables();
  private:
  const ::dtproto::geometry_msgs::Polygon3d& _internal_steppables(int index) const;
  ::dtproto::geometry_msgs::Polygon3d* _internal_add_steppables();
  public:
  const ::dtproto::geometry_msgs::Polygon3d& steppables(int index) const;
  ::dtproto::geometry_msgs::Polygon3d* add_steppables();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::dtproto::geometry_msgs::Polygon3d >&
      steppables() const;

  // repeated .dtproto.geometry_msgs.Polygon3d unsteppables = 3;
  int unsteppables_size() const;
  private:
  int _internal_unsteppables_size() const;
  public:
  void clear_unsteppables();
  ::dtproto::geometry_msgs::Polygon3d* mutable_unsteppables(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::dtproto::geometry_msgs::Polygon3d >*
      mutable_unsteppables();
  private:
  const ::dtproto::geometry_msgs::Polygon3d& _internal_unsteppables(int index) const;
  ::dtproto::geometry_msgs::Polygon3d* _internal_add_unsteppables();
  public:
  const ::dtproto::geometry_msgs::Polygon3d& unsteppables(int index) const;
  ::dtproto::geometry_msgs::Polygon3d* add_unsteppables();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::dtproto::geometry_msgs::Polygon3d >&
      unsteppables() const;

  // uint32 steppables_count = 2;
  void clear_steppables_count();
  uint32_t steppables_count() const;
  void set_steppables_count(uint32_t value);
  private:
  uint32_t _internal_steppables_count() const;
  void _internal_set_steppables_count(uint32_t value);
  public:

  // uint32 unsteppables_count = 4;
  void clear_unsteppables_count();
  uint32_t unsteppables_count() const;
  void set_unsteppables_count(uint32_t value);
  private:
  uint32_t _internal_unsteppables_count() const;
  void _internal_set_unsteppables_count(uint32_t value);
  public:

  // @@protoc_insertion_point(class_scope:dtproto.nav_msgs.SteppableArea)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  struct Impl_ {
    ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::dtproto::geometry_msgs::Polygon3d > steppables_;
    ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::dtproto::geometry_msgs::Polygon3d > unsteppables_;
    uint32_t steppables_count_;
    uint32_t unsteppables_count_;
    mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  };
  union { Impl_ _impl_; };
  friend struct ::TableStruct_dtProto_2fnav_5fmsgs_2fSteppableArea_2eproto;
};
// -------------------------------------------------------------------

class SteppableAreaTimeStamped final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:dtproto.nav_msgs.SteppableAreaTimeStamped) */ {
 public:
  inline SteppableAreaTimeStamped() : SteppableAreaTimeStamped(nullptr) {}
  ~SteppableAreaTimeStamped() override;
  explicit PROTOBUF_CONSTEXPR SteppableAreaTimeStamped(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  SteppableAreaTimeStamped(const SteppableAreaTimeStamped& from);
  SteppableAreaTimeStamped(SteppableAreaTimeStamped&& from) noexcept
    : SteppableAreaTimeStamped() {
    *this = ::std::move(from);
  }

  inline SteppableAreaTimeStamped& operator=(const SteppableAreaTimeStamped& from) {
    CopyFrom(from);
    return *this;
  }
  inline SteppableAreaTimeStamped& operator=(SteppableAreaTimeStamped&& from) noexcept {
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
  static const SteppableAreaTimeStamped& default_instance() {
    return *internal_default_instance();
  }
  static inline const SteppableAreaTimeStamped* internal_default_instance() {
    return reinterpret_cast<const SteppableAreaTimeStamped*>(
               &_SteppableAreaTimeStamped_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(SteppableAreaTimeStamped& a, SteppableAreaTimeStamped& b) {
    a.Swap(&b);
  }
  inline void Swap(SteppableAreaTimeStamped* other) {
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
  void UnsafeArenaSwap(SteppableAreaTimeStamped* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  SteppableAreaTimeStamped* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<SteppableAreaTimeStamped>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const SteppableAreaTimeStamped& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom( const SteppableAreaTimeStamped& from) {
    SteppableAreaTimeStamped::MergeImpl(*this, from);
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
  void InternalSwap(SteppableAreaTimeStamped* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "dtproto.nav_msgs.SteppableAreaTimeStamped";
  }
  protected:
  explicit SteppableAreaTimeStamped(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kHeaderFieldNumber = 1,
    kAreaFieldNumber = 2,
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

  // .dtproto.nav_msgs.SteppableArea area = 2;
  bool has_area() const;
  private:
  bool _internal_has_area() const;
  public:
  void clear_area();
  const ::dtproto::nav_msgs::SteppableArea& area() const;
  PROTOBUF_NODISCARD ::dtproto::nav_msgs::SteppableArea* release_area();
  ::dtproto::nav_msgs::SteppableArea* mutable_area();
  void set_allocated_area(::dtproto::nav_msgs::SteppableArea* area);
  private:
  const ::dtproto::nav_msgs::SteppableArea& _internal_area() const;
  ::dtproto::nav_msgs::SteppableArea* _internal_mutable_area();
  public:
  void unsafe_arena_set_allocated_area(
      ::dtproto::nav_msgs::SteppableArea* area);
  ::dtproto::nav_msgs::SteppableArea* unsafe_arena_release_area();

  // @@protoc_insertion_point(class_scope:dtproto.nav_msgs.SteppableAreaTimeStamped)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  struct Impl_ {
    ::dtproto::std_msgs::Header* header_;
    ::dtproto::nav_msgs::SteppableArea* area_;
    mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  };
  union { Impl_ _impl_; };
  friend struct ::TableStruct_dtProto_2fnav_5fmsgs_2fSteppableArea_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// SteppableArea

// repeated .dtproto.geometry_msgs.Polygon3d steppables = 1;
inline int SteppableArea::_internal_steppables_size() const {
  return _impl_.steppables_.size();
}
inline int SteppableArea::steppables_size() const {
  return _internal_steppables_size();
}
inline ::dtproto::geometry_msgs::Polygon3d* SteppableArea::mutable_steppables(int index) {
  // @@protoc_insertion_point(field_mutable:dtproto.nav_msgs.SteppableArea.steppables)
  return _impl_.steppables_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::dtproto::geometry_msgs::Polygon3d >*
SteppableArea::mutable_steppables() {
  // @@protoc_insertion_point(field_mutable_list:dtproto.nav_msgs.SteppableArea.steppables)
  return &_impl_.steppables_;
}
inline const ::dtproto::geometry_msgs::Polygon3d& SteppableArea::_internal_steppables(int index) const {
  return _impl_.steppables_.Get(index);
}
inline const ::dtproto::geometry_msgs::Polygon3d& SteppableArea::steppables(int index) const {
  // @@protoc_insertion_point(field_get:dtproto.nav_msgs.SteppableArea.steppables)
  return _internal_steppables(index);
}
inline ::dtproto::geometry_msgs::Polygon3d* SteppableArea::_internal_add_steppables() {
  return _impl_.steppables_.Add();
}
inline ::dtproto::geometry_msgs::Polygon3d* SteppableArea::add_steppables() {
  ::dtproto::geometry_msgs::Polygon3d* _add = _internal_add_steppables();
  // @@protoc_insertion_point(field_add:dtproto.nav_msgs.SteppableArea.steppables)
  return _add;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::dtproto::geometry_msgs::Polygon3d >&
SteppableArea::steppables() const {
  // @@protoc_insertion_point(field_list:dtproto.nav_msgs.SteppableArea.steppables)
  return _impl_.steppables_;
}

// uint32 steppables_count = 2;
inline void SteppableArea::clear_steppables_count() {
  _impl_.steppables_count_ = 0u;
}
inline uint32_t SteppableArea::_internal_steppables_count() const {
  return _impl_.steppables_count_;
}
inline uint32_t SteppableArea::steppables_count() const {
  // @@protoc_insertion_point(field_get:dtproto.nav_msgs.SteppableArea.steppables_count)
  return _internal_steppables_count();
}
inline void SteppableArea::_internal_set_steppables_count(uint32_t value) {
  
  _impl_.steppables_count_ = value;
}
inline void SteppableArea::set_steppables_count(uint32_t value) {
  _internal_set_steppables_count(value);
  // @@protoc_insertion_point(field_set:dtproto.nav_msgs.SteppableArea.steppables_count)
}

// repeated .dtproto.geometry_msgs.Polygon3d unsteppables = 3;
inline int SteppableArea::_internal_unsteppables_size() const {
  return _impl_.unsteppables_.size();
}
inline int SteppableArea::unsteppables_size() const {
  return _internal_unsteppables_size();
}
inline ::dtproto::geometry_msgs::Polygon3d* SteppableArea::mutable_unsteppables(int index) {
  // @@protoc_insertion_point(field_mutable:dtproto.nav_msgs.SteppableArea.unsteppables)
  return _impl_.unsteppables_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::dtproto::geometry_msgs::Polygon3d >*
SteppableArea::mutable_unsteppables() {
  // @@protoc_insertion_point(field_mutable_list:dtproto.nav_msgs.SteppableArea.unsteppables)
  return &_impl_.unsteppables_;
}
inline const ::dtproto::geometry_msgs::Polygon3d& SteppableArea::_internal_unsteppables(int index) const {
  return _impl_.unsteppables_.Get(index);
}
inline const ::dtproto::geometry_msgs::Polygon3d& SteppableArea::unsteppables(int index) const {
  // @@protoc_insertion_point(field_get:dtproto.nav_msgs.SteppableArea.unsteppables)
  return _internal_unsteppables(index);
}
inline ::dtproto::geometry_msgs::Polygon3d* SteppableArea::_internal_add_unsteppables() {
  return _impl_.unsteppables_.Add();
}
inline ::dtproto::geometry_msgs::Polygon3d* SteppableArea::add_unsteppables() {
  ::dtproto::geometry_msgs::Polygon3d* _add = _internal_add_unsteppables();
  // @@protoc_insertion_point(field_add:dtproto.nav_msgs.SteppableArea.unsteppables)
  return _add;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::dtproto::geometry_msgs::Polygon3d >&
SteppableArea::unsteppables() const {
  // @@protoc_insertion_point(field_list:dtproto.nav_msgs.SteppableArea.unsteppables)
  return _impl_.unsteppables_;
}

// uint32 unsteppables_count = 4;
inline void SteppableArea::clear_unsteppables_count() {
  _impl_.unsteppables_count_ = 0u;
}
inline uint32_t SteppableArea::_internal_unsteppables_count() const {
  return _impl_.unsteppables_count_;
}
inline uint32_t SteppableArea::unsteppables_count() const {
  // @@protoc_insertion_point(field_get:dtproto.nav_msgs.SteppableArea.unsteppables_count)
  return _internal_unsteppables_count();
}
inline void SteppableArea::_internal_set_unsteppables_count(uint32_t value) {
  
  _impl_.unsteppables_count_ = value;
}
inline void SteppableArea::set_unsteppables_count(uint32_t value) {
  _internal_set_unsteppables_count(value);
  // @@protoc_insertion_point(field_set:dtproto.nav_msgs.SteppableArea.unsteppables_count)
}

// -------------------------------------------------------------------

// SteppableAreaTimeStamped

// .dtproto.std_msgs.Header header = 1;
inline bool SteppableAreaTimeStamped::_internal_has_header() const {
  return this != internal_default_instance() && _impl_.header_ != nullptr;
}
inline bool SteppableAreaTimeStamped::has_header() const {
  return _internal_has_header();
}
inline const ::dtproto::std_msgs::Header& SteppableAreaTimeStamped::_internal_header() const {
  const ::dtproto::std_msgs::Header* p = _impl_.header_;
  return p != nullptr ? *p : reinterpret_cast<const ::dtproto::std_msgs::Header&>(
      ::dtproto::std_msgs::_Header_default_instance_);
}
inline const ::dtproto::std_msgs::Header& SteppableAreaTimeStamped::header() const {
  // @@protoc_insertion_point(field_get:dtproto.nav_msgs.SteppableAreaTimeStamped.header)
  return _internal_header();
}
inline void SteppableAreaTimeStamped::unsafe_arena_set_allocated_header(
    ::dtproto::std_msgs::Header* header) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.header_);
  }
  _impl_.header_ = header;
  if (header) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:dtproto.nav_msgs.SteppableAreaTimeStamped.header)
}
inline ::dtproto::std_msgs::Header* SteppableAreaTimeStamped::release_header() {
  
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
inline ::dtproto::std_msgs::Header* SteppableAreaTimeStamped::unsafe_arena_release_header() {
  // @@protoc_insertion_point(field_release:dtproto.nav_msgs.SteppableAreaTimeStamped.header)
  
  ::dtproto::std_msgs::Header* temp = _impl_.header_;
  _impl_.header_ = nullptr;
  return temp;
}
inline ::dtproto::std_msgs::Header* SteppableAreaTimeStamped::_internal_mutable_header() {
  
  if (_impl_.header_ == nullptr) {
    auto* p = CreateMaybeMessage<::dtproto::std_msgs::Header>(GetArenaForAllocation());
    _impl_.header_ = p;
  }
  return _impl_.header_;
}
inline ::dtproto::std_msgs::Header* SteppableAreaTimeStamped::mutable_header() {
  ::dtproto::std_msgs::Header* _msg = _internal_mutable_header();
  // @@protoc_insertion_point(field_mutable:dtproto.nav_msgs.SteppableAreaTimeStamped.header)
  return _msg;
}
inline void SteppableAreaTimeStamped::set_allocated_header(::dtproto::std_msgs::Header* header) {
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
  // @@protoc_insertion_point(field_set_allocated:dtproto.nav_msgs.SteppableAreaTimeStamped.header)
}

// .dtproto.nav_msgs.SteppableArea area = 2;
inline bool SteppableAreaTimeStamped::_internal_has_area() const {
  return this != internal_default_instance() && _impl_.area_ != nullptr;
}
inline bool SteppableAreaTimeStamped::has_area() const {
  return _internal_has_area();
}
inline void SteppableAreaTimeStamped::clear_area() {
  if (GetArenaForAllocation() == nullptr && _impl_.area_ != nullptr) {
    delete _impl_.area_;
  }
  _impl_.area_ = nullptr;
}
inline const ::dtproto::nav_msgs::SteppableArea& SteppableAreaTimeStamped::_internal_area() const {
  const ::dtproto::nav_msgs::SteppableArea* p = _impl_.area_;
  return p != nullptr ? *p : reinterpret_cast<const ::dtproto::nav_msgs::SteppableArea&>(
      ::dtproto::nav_msgs::_SteppableArea_default_instance_);
}
inline const ::dtproto::nav_msgs::SteppableArea& SteppableAreaTimeStamped::area() const {
  // @@protoc_insertion_point(field_get:dtproto.nav_msgs.SteppableAreaTimeStamped.area)
  return _internal_area();
}
inline void SteppableAreaTimeStamped::unsafe_arena_set_allocated_area(
    ::dtproto::nav_msgs::SteppableArea* area) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.area_);
  }
  _impl_.area_ = area;
  if (area) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:dtproto.nav_msgs.SteppableAreaTimeStamped.area)
}
inline ::dtproto::nav_msgs::SteppableArea* SteppableAreaTimeStamped::release_area() {
  
  ::dtproto::nav_msgs::SteppableArea* temp = _impl_.area_;
  _impl_.area_ = nullptr;
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
inline ::dtproto::nav_msgs::SteppableArea* SteppableAreaTimeStamped::unsafe_arena_release_area() {
  // @@protoc_insertion_point(field_release:dtproto.nav_msgs.SteppableAreaTimeStamped.area)
  
  ::dtproto::nav_msgs::SteppableArea* temp = _impl_.area_;
  _impl_.area_ = nullptr;
  return temp;
}
inline ::dtproto::nav_msgs::SteppableArea* SteppableAreaTimeStamped::_internal_mutable_area() {
  
  if (_impl_.area_ == nullptr) {
    auto* p = CreateMaybeMessage<::dtproto::nav_msgs::SteppableArea>(GetArenaForAllocation());
    _impl_.area_ = p;
  }
  return _impl_.area_;
}
inline ::dtproto::nav_msgs::SteppableArea* SteppableAreaTimeStamped::mutable_area() {
  ::dtproto::nav_msgs::SteppableArea* _msg = _internal_mutable_area();
  // @@protoc_insertion_point(field_mutable:dtproto.nav_msgs.SteppableAreaTimeStamped.area)
  return _msg;
}
inline void SteppableAreaTimeStamped::set_allocated_area(::dtproto::nav_msgs::SteppableArea* area) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete _impl_.area_;
  }
  if (area) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalGetOwningArena(area);
    if (message_arena != submessage_arena) {
      area = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, area, submessage_arena);
    }
    
  } else {
    
  }
  _impl_.area_ = area;
  // @@protoc_insertion_point(field_set_allocated:dtproto.nav_msgs.SteppableAreaTimeStamped.area)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace nav_msgs
}  // namespace dtproto

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_dtProto_2fnav_5fmsgs_2fSteppableArea_2eproto