// automatically generated by the FlatBuffers compiler, do not modify


#ifndef FLATBUFFERS_GENERATED_RECORDING_FLATBUFFER_H_
#define FLATBUFFERS_GENERATED_RECORDING_FLATBUFFER_H_

#include "flatbuffers/flatbuffers.h"

// Ensure the included flatbuffers.h is the same version as when this file was
// generated, otherwise it may not be compatible.
static_assert(FLATBUFFERS_VERSION_MAJOR == 24 &&
              FLATBUFFERS_VERSION_MINOR == 3 &&
              FLATBUFFERS_VERSION_REVISION == 25,
             "Non-compatible flatbuffers version included");

#include "pose_generated.h"

namespace flatbuffer {

struct Observation;
struct ObservationBuilder;
struct ObservationT;

struct Recording;
struct RecordingBuilder;
struct RecordingT;

struct Image;
struct ImageBuilder;
struct ImageT;

struct ObservationT : public ::flatbuffers::NativeTable {
  typedef Observation TableType;
  std::unique_ptr<flatbuffer::Pose> qpos{};
  std::unique_ptr<flatbuffer::Pose> action{};
  std::vector<std::unique_ptr<flatbuffer::ImageT>> cameras{};
  ObservationT() = default;
  ObservationT(const ObservationT &o);
  ObservationT(ObservationT&&) FLATBUFFERS_NOEXCEPT = default;
  ObservationT &operator=(ObservationT o) FLATBUFFERS_NOEXCEPT;
};

struct Observation FLATBUFFERS_FINAL_CLASS : private ::flatbuffers::Table {
  typedef ObservationT NativeTableType;
  typedef ObservationBuilder Builder;
  struct Traits;
  enum FlatBuffersVTableOffset FLATBUFFERS_VTABLE_UNDERLYING_TYPE {
    VT_QPOS = 8,
    VT_ACTION = 10,
    VT_CAMERAS = 12
  };
  const flatbuffer::Pose *qpos() const {
    return GetStruct<const flatbuffer::Pose *>(VT_QPOS);
  }
  const flatbuffer::Pose *action() const {
    return GetStruct<const flatbuffer::Pose *>(VT_ACTION);
  }
  const ::flatbuffers::Vector<::flatbuffers::Offset<flatbuffer::Image>> *cameras() const {
    return GetPointer<const ::flatbuffers::Vector<::flatbuffers::Offset<flatbuffer::Image>> *>(VT_CAMERAS);
  }
  bool Verify(::flatbuffers::Verifier &verifier) const {
    return VerifyTableStart(verifier) &&
           VerifyField<flatbuffer::Pose>(verifier, VT_QPOS, 8) &&
           VerifyField<flatbuffer::Pose>(verifier, VT_ACTION, 8) &&
           VerifyOffset(verifier, VT_CAMERAS) &&
           verifier.VerifyVector(cameras()) &&
           verifier.VerifyVectorOfTables(cameras()) &&
           verifier.EndTable();
  }
  ObservationT *UnPack(const ::flatbuffers::resolver_function_t *_resolver = nullptr) const;
  void UnPackTo(ObservationT *_o, const ::flatbuffers::resolver_function_t *_resolver = nullptr) const;
  static ::flatbuffers::Offset<Observation> Pack(::flatbuffers::FlatBufferBuilder &_fbb, const ObservationT* _o, const ::flatbuffers::rehasher_function_t *_rehasher = nullptr);
};

struct ObservationBuilder {
  typedef Observation Table;
  ::flatbuffers::FlatBufferBuilder &fbb_;
  ::flatbuffers::uoffset_t start_;
  void add_qpos(const flatbuffer::Pose *qpos) {
    fbb_.AddStruct(Observation::VT_QPOS, qpos);
  }
  void add_action(const flatbuffer::Pose *action) {
    fbb_.AddStruct(Observation::VT_ACTION, action);
  }
  void add_cameras(::flatbuffers::Offset<::flatbuffers::Vector<::flatbuffers::Offset<flatbuffer::Image>>> cameras) {
    fbb_.AddOffset(Observation::VT_CAMERAS, cameras);
  }
  explicit ObservationBuilder(::flatbuffers::FlatBufferBuilder &_fbb)
        : fbb_(_fbb) {
    start_ = fbb_.StartTable();
  }
  ::flatbuffers::Offset<Observation> Finish() {
    const auto end = fbb_.EndTable(start_);
    auto o = ::flatbuffers::Offset<Observation>(end);
    return o;
  }
};

inline ::flatbuffers::Offset<Observation> CreateObservation(
    ::flatbuffers::FlatBufferBuilder &_fbb,
    const flatbuffer::Pose *qpos = nullptr,
    const flatbuffer::Pose *action = nullptr,
    ::flatbuffers::Offset<::flatbuffers::Vector<::flatbuffers::Offset<flatbuffer::Image>>> cameras = 0) {
  ObservationBuilder builder_(_fbb);
  builder_.add_cameras(cameras);
  builder_.add_action(action);
  builder_.add_qpos(qpos);
  return builder_.Finish();
}

struct Observation::Traits {
  using type = Observation;
  static auto constexpr Create = CreateObservation;
};

inline ::flatbuffers::Offset<Observation> CreateObservationDirect(
    ::flatbuffers::FlatBufferBuilder &_fbb,
    const flatbuffer::Pose *qpos = nullptr,
    const flatbuffer::Pose *action = nullptr,
    const std::vector<::flatbuffers::Offset<flatbuffer::Image>> *cameras = nullptr) {
  auto cameras__ = cameras ? _fbb.CreateVector<::flatbuffers::Offset<flatbuffer::Image>>(*cameras) : 0;
  return flatbuffer::CreateObservation(
      _fbb,
      qpos,
      action,
      cameras__);
}

::flatbuffers::Offset<Observation> CreateObservation(::flatbuffers::FlatBufferBuilder &_fbb, const ObservationT *_o, const ::flatbuffers::rehasher_function_t *_rehasher = nullptr);

struct RecordingT : public ::flatbuffers::NativeTable {
  typedef Recording TableType;
  std::vector<std::unique_ptr<flatbuffer::ObservationT>> observations{};
  RecordingT() = default;
  RecordingT(const RecordingT &o);
  RecordingT(RecordingT&&) FLATBUFFERS_NOEXCEPT = default;
  RecordingT &operator=(RecordingT o) FLATBUFFERS_NOEXCEPT;
};

struct Recording FLATBUFFERS_FINAL_CLASS : private ::flatbuffers::Table {
  typedef RecordingT NativeTableType;
  typedef RecordingBuilder Builder;
  struct Traits;
  enum FlatBuffersVTableOffset FLATBUFFERS_VTABLE_UNDERLYING_TYPE {
    VT_OBSERVATIONS = 4
  };
  const ::flatbuffers::Vector<::flatbuffers::Offset<flatbuffer::Observation>> *observations() const {
    return GetPointer<const ::flatbuffers::Vector<::flatbuffers::Offset<flatbuffer::Observation>> *>(VT_OBSERVATIONS);
  }
  bool Verify(::flatbuffers::Verifier &verifier) const {
    return VerifyTableStart(verifier) &&
           VerifyOffset(verifier, VT_OBSERVATIONS) &&
           verifier.VerifyVector(observations()) &&
           verifier.VerifyVectorOfTables(observations()) &&
           verifier.EndTable();
  }
  RecordingT *UnPack(const ::flatbuffers::resolver_function_t *_resolver = nullptr) const;
  void UnPackTo(RecordingT *_o, const ::flatbuffers::resolver_function_t *_resolver = nullptr) const;
  static ::flatbuffers::Offset<Recording> Pack(::flatbuffers::FlatBufferBuilder &_fbb, const RecordingT* _o, const ::flatbuffers::rehasher_function_t *_rehasher = nullptr);
};

struct RecordingBuilder {
  typedef Recording Table;
  ::flatbuffers::FlatBufferBuilder &fbb_;
  ::flatbuffers::uoffset_t start_;
  void add_observations(::flatbuffers::Offset<::flatbuffers::Vector<::flatbuffers::Offset<flatbuffer::Observation>>> observations) {
    fbb_.AddOffset(Recording::VT_OBSERVATIONS, observations);
  }
  explicit RecordingBuilder(::flatbuffers::FlatBufferBuilder &_fbb)
        : fbb_(_fbb) {
    start_ = fbb_.StartTable();
  }
  ::flatbuffers::Offset<Recording> Finish() {
    const auto end = fbb_.EndTable(start_);
    auto o = ::flatbuffers::Offset<Recording>(end);
    return o;
  }
};

inline ::flatbuffers::Offset<Recording> CreateRecording(
    ::flatbuffers::FlatBufferBuilder &_fbb,
    ::flatbuffers::Offset<::flatbuffers::Vector<::flatbuffers::Offset<flatbuffer::Observation>>> observations = 0) {
  RecordingBuilder builder_(_fbb);
  builder_.add_observations(observations);
  return builder_.Finish();
}

struct Recording::Traits {
  using type = Recording;
  static auto constexpr Create = CreateRecording;
};

inline ::flatbuffers::Offset<Recording> CreateRecordingDirect(
    ::flatbuffers::FlatBufferBuilder &_fbb,
    const std::vector<::flatbuffers::Offset<flatbuffer::Observation>> *observations = nullptr) {
  auto observations__ = observations ? _fbb.CreateVector<::flatbuffers::Offset<flatbuffer::Observation>>(*observations) : 0;
  return flatbuffer::CreateRecording(
      _fbb,
      observations__);
}

::flatbuffers::Offset<Recording> CreateRecording(::flatbuffers::FlatBufferBuilder &_fbb, const RecordingT *_o, const ::flatbuffers::rehasher_function_t *_rehasher = nullptr);

struct ImageT : public ::flatbuffers::NativeTable {
  typedef Image TableType;
  std::vector<uint8_t> data{};
};

struct Image FLATBUFFERS_FINAL_CLASS : private ::flatbuffers::Table {
  typedef ImageT NativeTableType;
  typedef ImageBuilder Builder;
  struct Traits;
  enum FlatBuffersVTableOffset FLATBUFFERS_VTABLE_UNDERLYING_TYPE {
    VT_DATA = 4
  };
  const ::flatbuffers::Vector<uint8_t> *data() const {
    return GetPointer<const ::flatbuffers::Vector<uint8_t> *>(VT_DATA);
  }
  bool Verify(::flatbuffers::Verifier &verifier) const {
    return VerifyTableStart(verifier) &&
           VerifyOffset(verifier, VT_DATA) &&
           verifier.VerifyVector(data()) &&
           verifier.EndTable();
  }
  ImageT *UnPack(const ::flatbuffers::resolver_function_t *_resolver = nullptr) const;
  void UnPackTo(ImageT *_o, const ::flatbuffers::resolver_function_t *_resolver = nullptr) const;
  static ::flatbuffers::Offset<Image> Pack(::flatbuffers::FlatBufferBuilder &_fbb, const ImageT* _o, const ::flatbuffers::rehasher_function_t *_rehasher = nullptr);
};

struct ImageBuilder {
  typedef Image Table;
  ::flatbuffers::FlatBufferBuilder &fbb_;
  ::flatbuffers::uoffset_t start_;
  void add_data(::flatbuffers::Offset<::flatbuffers::Vector<uint8_t>> data) {
    fbb_.AddOffset(Image::VT_DATA, data);
  }
  explicit ImageBuilder(::flatbuffers::FlatBufferBuilder &_fbb)
        : fbb_(_fbb) {
    start_ = fbb_.StartTable();
  }
  ::flatbuffers::Offset<Image> Finish() {
    const auto end = fbb_.EndTable(start_);
    auto o = ::flatbuffers::Offset<Image>(end);
    return o;
  }
};

inline ::flatbuffers::Offset<Image> CreateImage(
    ::flatbuffers::FlatBufferBuilder &_fbb,
    ::flatbuffers::Offset<::flatbuffers::Vector<uint8_t>> data = 0) {
  ImageBuilder builder_(_fbb);
  builder_.add_data(data);
  return builder_.Finish();
}

struct Image::Traits {
  using type = Image;
  static auto constexpr Create = CreateImage;
};

inline ::flatbuffers::Offset<Image> CreateImageDirect(
    ::flatbuffers::FlatBufferBuilder &_fbb,
    const std::vector<uint8_t> *data = nullptr) {
  auto data__ = data ? _fbb.CreateVector<uint8_t>(*data) : 0;
  return flatbuffer::CreateImage(
      _fbb,
      data__);
}

::flatbuffers::Offset<Image> CreateImage(::flatbuffers::FlatBufferBuilder &_fbb, const ImageT *_o, const ::flatbuffers::rehasher_function_t *_rehasher = nullptr);

inline ObservationT::ObservationT(const ObservationT &o)
      : qpos((o.qpos) ? new flatbuffer::Pose(*o.qpos) : nullptr),
        action((o.action) ? new flatbuffer::Pose(*o.action) : nullptr) {
  cameras.reserve(o.cameras.size());
  for (const auto &cameras_ : o.cameras) { cameras.emplace_back((cameras_) ? new flatbuffer::ImageT(*cameras_) : nullptr); }
}

inline ObservationT &ObservationT::operator=(ObservationT o) FLATBUFFERS_NOEXCEPT {
  std::swap(qpos, o.qpos);
  std::swap(action, o.action);
  std::swap(cameras, o.cameras);
  return *this;
}

inline ObservationT *Observation::UnPack(const ::flatbuffers::resolver_function_t *_resolver) const {
  auto _o = std::make_unique<ObservationT>();
  UnPackTo(_o.get(), _resolver);
  return _o.release();
}

inline void Observation::UnPackTo(ObservationT *_o, const ::flatbuffers::resolver_function_t *_resolver) const {
  (void)_o;
  (void)_resolver;
  { auto _e = qpos(); if (_e) _o->qpos = std::unique_ptr<flatbuffer::Pose>(new flatbuffer::Pose(*_e)); }
  { auto _e = action(); if (_e) _o->action = std::unique_ptr<flatbuffer::Pose>(new flatbuffer::Pose(*_e)); }
  { auto _e = cameras(); if (_e) { _o->cameras.resize(_e->size()); for (::flatbuffers::uoffset_t _i = 0; _i < _e->size(); _i++) { if(_o->cameras[_i]) { _e->Get(_i)->UnPackTo(_o->cameras[_i].get(), _resolver); } else { _o->cameras[_i] = std::unique_ptr<flatbuffer::ImageT>(_e->Get(_i)->UnPack(_resolver)); }; } } else { _o->cameras.resize(0); } }
}

inline ::flatbuffers::Offset<Observation> Observation::Pack(::flatbuffers::FlatBufferBuilder &_fbb, const ObservationT* _o, const ::flatbuffers::rehasher_function_t *_rehasher) {
  return CreateObservation(_fbb, _o, _rehasher);
}

inline ::flatbuffers::Offset<Observation> CreateObservation(::flatbuffers::FlatBufferBuilder &_fbb, const ObservationT *_o, const ::flatbuffers::rehasher_function_t *_rehasher) {
  (void)_rehasher;
  (void)_o;
  struct _VectorArgs { ::flatbuffers::FlatBufferBuilder *__fbb; const ObservationT* __o; const ::flatbuffers::rehasher_function_t *__rehasher; } _va = { &_fbb, _o, _rehasher}; (void)_va;
  auto _qpos = _o->qpos ? _o->qpos.get() : nullptr;
  auto _action = _o->action ? _o->action.get() : nullptr;
  auto _cameras = _o->cameras.size() ? _fbb.CreateVector<::flatbuffers::Offset<flatbuffer::Image>> (_o->cameras.size(), [](size_t i, _VectorArgs *__va) { return CreateImage(*__va->__fbb, __va->__o->cameras[i].get(), __va->__rehasher); }, &_va ) : 0;
  return flatbuffer::CreateObservation(
      _fbb,
      _qpos,
      _action,
      _cameras);
}

inline RecordingT::RecordingT(const RecordingT &o) {
  observations.reserve(o.observations.size());
  for (const auto &observations_ : o.observations) { observations.emplace_back((observations_) ? new flatbuffer::ObservationT(*observations_) : nullptr); }
}

inline RecordingT &RecordingT::operator=(RecordingT o) FLATBUFFERS_NOEXCEPT {
  std::swap(observations, o.observations);
  return *this;
}

inline RecordingT *Recording::UnPack(const ::flatbuffers::resolver_function_t *_resolver) const {
  auto _o = std::make_unique<RecordingT>();
  UnPackTo(_o.get(), _resolver);
  return _o.release();
}

inline void Recording::UnPackTo(RecordingT *_o, const ::flatbuffers::resolver_function_t *_resolver) const {
  (void)_o;
  (void)_resolver;
  { auto _e = observations(); if (_e) { _o->observations.resize(_e->size()); for (::flatbuffers::uoffset_t _i = 0; _i < _e->size(); _i++) { if(_o->observations[_i]) { _e->Get(_i)->UnPackTo(_o->observations[_i].get(), _resolver); } else { _o->observations[_i] = std::unique_ptr<flatbuffer::ObservationT>(_e->Get(_i)->UnPack(_resolver)); }; } } else { _o->observations.resize(0); } }
}

inline ::flatbuffers::Offset<Recording> Recording::Pack(::flatbuffers::FlatBufferBuilder &_fbb, const RecordingT* _o, const ::flatbuffers::rehasher_function_t *_rehasher) {
  return CreateRecording(_fbb, _o, _rehasher);
}

inline ::flatbuffers::Offset<Recording> CreateRecording(::flatbuffers::FlatBufferBuilder &_fbb, const RecordingT *_o, const ::flatbuffers::rehasher_function_t *_rehasher) {
  (void)_rehasher;
  (void)_o;
  struct _VectorArgs { ::flatbuffers::FlatBufferBuilder *__fbb; const RecordingT* __o; const ::flatbuffers::rehasher_function_t *__rehasher; } _va = { &_fbb, _o, _rehasher}; (void)_va;
  auto _observations = _o->observations.size() ? _fbb.CreateVector<::flatbuffers::Offset<flatbuffer::Observation>> (_o->observations.size(), [](size_t i, _VectorArgs *__va) { return CreateObservation(*__va->__fbb, __va->__o->observations[i].get(), __va->__rehasher); }, &_va ) : 0;
  return flatbuffer::CreateRecording(
      _fbb,
      _observations);
}

inline ImageT *Image::UnPack(const ::flatbuffers::resolver_function_t *_resolver) const {
  auto _o = std::make_unique<ImageT>();
  UnPackTo(_o.get(), _resolver);
  return _o.release();
}

inline void Image::UnPackTo(ImageT *_o, const ::flatbuffers::resolver_function_t *_resolver) const {
  (void)_o;
  (void)_resolver;
  { auto _e = data(); if (_e) { _o->data.resize(_e->size()); std::copy(_e->begin(), _e->end(), _o->data.begin()); } }
}

inline ::flatbuffers::Offset<Image> Image::Pack(::flatbuffers::FlatBufferBuilder &_fbb, const ImageT* _o, const ::flatbuffers::rehasher_function_t *_rehasher) {
  return CreateImage(_fbb, _o, _rehasher);
}

inline ::flatbuffers::Offset<Image> CreateImage(::flatbuffers::FlatBufferBuilder &_fbb, const ImageT *_o, const ::flatbuffers::rehasher_function_t *_rehasher) {
  (void)_rehasher;
  (void)_o;
  struct _VectorArgs { ::flatbuffers::FlatBufferBuilder *__fbb; const ImageT* __o; const ::flatbuffers::rehasher_function_t *__rehasher; } _va = { &_fbb, _o, _rehasher}; (void)_va;
  auto _data = _o->data.size() ? _fbb.CreateVector(_o->data) : 0;
  return flatbuffer::CreateImage(
      _fbb,
      _data);
}

inline const flatbuffer::Recording *GetRecording(const void *buf) {
  return ::flatbuffers::GetRoot<flatbuffer::Recording>(buf);
}

inline const flatbuffer::Recording *GetSizePrefixedRecording(const void *buf) {
  return ::flatbuffers::GetSizePrefixedRoot<flatbuffer::Recording>(buf);
}

inline bool VerifyRecordingBuffer(
    ::flatbuffers::Verifier &verifier) {
  return verifier.VerifyBuffer<flatbuffer::Recording>(nullptr);
}

inline bool VerifySizePrefixedRecordingBuffer(
    ::flatbuffers::Verifier &verifier) {
  return verifier.VerifySizePrefixedBuffer<flatbuffer::Recording>(nullptr);
}

inline void FinishRecordingBuffer(
    ::flatbuffers::FlatBufferBuilder &fbb,
    ::flatbuffers::Offset<flatbuffer::Recording> root) {
  fbb.Finish(root);
}

inline void FinishSizePrefixedRecordingBuffer(
    ::flatbuffers::FlatBufferBuilder &fbb,
    ::flatbuffers::Offset<flatbuffer::Recording> root) {
  fbb.FinishSizePrefixed(root);
}

inline std::unique_ptr<flatbuffer::RecordingT> UnPackRecording(
    const void *buf,
    const ::flatbuffers::resolver_function_t *res = nullptr) {
  return std::unique_ptr<flatbuffer::RecordingT>(GetRecording(buf)->UnPack(res));
}

inline std::unique_ptr<flatbuffer::RecordingT> UnPackSizePrefixedRecording(
    const void *buf,
    const ::flatbuffers::resolver_function_t *res = nullptr) {
  return std::unique_ptr<flatbuffer::RecordingT>(GetSizePrefixedRecording(buf)->UnPack(res));
}

}  // namespace flatbuffer

#endif  // FLATBUFFERS_GENERATED_RECORDING_FLATBUFFER_H_