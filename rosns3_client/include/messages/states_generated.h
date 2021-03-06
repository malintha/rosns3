// automatically generated by the FlatBuffers compiler, do not modify


#ifndef FLATBUFFERS_GENERATED_STATES_H_
#define FLATBUFFERS_GENERATED_STATES_H_

#include "flatbuffers/flatbuffers.h"

struct Vec3;

struct Agent;
struct AgentBuilder;

struct Swarm;
struct SwarmBuilder;

FLATBUFFERS_MANUALLY_ALIGNED_STRUCT(4) Vec3 FLATBUFFERS_FINAL_CLASS {
 private:
  float x_;
  float y_;
  float z_;

 public:
  Vec3()
      : x_(0),
        y_(0),
        z_(0) {
  }
  Vec3(float _x, float _y, float _z)
      : x_(flatbuffers::EndianScalar(_x)),
        y_(flatbuffers::EndianScalar(_y)),
        z_(flatbuffers::EndianScalar(_z)) {
  }
  float x() const {
    return flatbuffers::EndianScalar(x_);
  }
  float y() const {
    return flatbuffers::EndianScalar(y_);
  }
  float z() const {
    return flatbuffers::EndianScalar(z_);
  }
};
FLATBUFFERS_STRUCT_END(Vec3, 12);

struct Agent FLATBUFFERS_FINAL_CLASS : private flatbuffers::Table {
  typedef AgentBuilder Builder;
  enum FlatBuffersVTableOffset FLATBUFFERS_VTABLE_UNDERLYING_TYPE {
    VT_POSITION = 4,
    VT_ID = 6
  };
  const Vec3 *position() const {
    return GetStruct<const Vec3 *>(VT_POSITION);
  }
  int32_t id() const {
    return GetField<int32_t>(VT_ID, 0);
  }
  bool Verify(flatbuffers::Verifier &verifier) const {
    return VerifyTableStart(verifier) &&
           VerifyField<Vec3>(verifier, VT_POSITION) &&
           VerifyField<int32_t>(verifier, VT_ID) &&
           verifier.EndTable();
  }
};

struct AgentBuilder {
  typedef Agent Table;
  flatbuffers::FlatBufferBuilder &fbb_;
  flatbuffers::uoffset_t start_;
  void add_position(const Vec3 *position) {
    fbb_.AddStruct(Agent::VT_POSITION, position);
  }
  void add_id(int32_t id) {
    fbb_.AddElement<int32_t>(Agent::VT_ID, id, 0);
  }
  explicit AgentBuilder(flatbuffers::FlatBufferBuilder &_fbb)
        : fbb_(_fbb) {
    start_ = fbb_.StartTable();
  }
  flatbuffers::Offset<Agent> Finish() {
    const auto end = fbb_.EndTable(start_);
    auto o = flatbuffers::Offset<Agent>(end);
    return o;
  }
};

inline flatbuffers::Offset<Agent> CreateAgent(
    flatbuffers::FlatBufferBuilder &_fbb,
    const Vec3 *position = 0,
    int32_t id = 0) {
  AgentBuilder builder_(_fbb);
  builder_.add_id(id);
  builder_.add_position(position);
  return builder_.Finish();
}

struct Swarm FLATBUFFERS_FINAL_CLASS : private flatbuffers::Table {
  typedef SwarmBuilder Builder;
  enum FlatBuffersVTableOffset FLATBUFFERS_VTABLE_UNDERLYING_TYPE {
    VT_BACKBONE = 4,
    VT_AGENTS = 6
  };
  int32_t backbone() const {
    return GetField<int32_t>(VT_BACKBONE, 0);
  }
  const flatbuffers::Vector<flatbuffers::Offset<Agent>> *agents() const {
    return GetPointer<const flatbuffers::Vector<flatbuffers::Offset<Agent>> *>(VT_AGENTS);
  }
  bool Verify(flatbuffers::Verifier &verifier) const {
    return VerifyTableStart(verifier) &&
           VerifyField<int32_t>(verifier, VT_BACKBONE) &&
           VerifyOffset(verifier, VT_AGENTS) &&
           verifier.VerifyVector(agents()) &&
           verifier.VerifyVectorOfTables(agents()) &&
           verifier.EndTable();
  }
};

struct SwarmBuilder {
  typedef Swarm Table;
  flatbuffers::FlatBufferBuilder &fbb_;
  flatbuffers::uoffset_t start_;
  void add_backbone(int32_t backbone) {
    fbb_.AddElement<int32_t>(Swarm::VT_BACKBONE, backbone, 0);
  }
  void add_agents(flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Agent>>> agents) {
    fbb_.AddOffset(Swarm::VT_AGENTS, agents);
  }
  explicit SwarmBuilder(flatbuffers::FlatBufferBuilder &_fbb)
        : fbb_(_fbb) {
    start_ = fbb_.StartTable();
  }
  flatbuffers::Offset<Swarm> Finish() {
    const auto end = fbb_.EndTable(start_);
    auto o = flatbuffers::Offset<Swarm>(end);
    return o;
  }
};

inline flatbuffers::Offset<Swarm> CreateSwarm(
    flatbuffers::FlatBufferBuilder &_fbb,
    int32_t backbone = 0,
    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Agent>>> agents = 0) {
  SwarmBuilder builder_(_fbb);
  builder_.add_agents(agents);
  builder_.add_backbone(backbone);
  return builder_.Finish();
}

inline flatbuffers::Offset<Swarm> CreateSwarmDirect(
    flatbuffers::FlatBufferBuilder &_fbb,
    int32_t backbone = 0,
    const std::vector<flatbuffers::Offset<Agent>> *agents = nullptr) {
  auto agents__ = agents ? _fbb.CreateVector<flatbuffers::Offset<Agent>>(*agents) : 0;
  return CreateSwarm(
      _fbb,
      backbone,
      agents__);
}

inline const Swarm *GetSwarm(const void *buf) {
  return flatbuffers::GetRoot<Swarm>(buf);
}

inline const Swarm *GetSizePrefixedSwarm(const void *buf) {
  return flatbuffers::GetSizePrefixedRoot<Swarm>(buf);
}

inline bool VerifySwarmBuffer(
    flatbuffers::Verifier &verifier) {
  return verifier.VerifyBuffer<Swarm>(nullptr);
}

inline bool VerifySizePrefixedSwarmBuffer(
    flatbuffers::Verifier &verifier) {
  return verifier.VerifySizePrefixedBuffer<Swarm>(nullptr);
}

inline void FinishSwarmBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    flatbuffers::Offset<Swarm> root) {
  fbb.Finish(root);
}

inline void FinishSizePrefixedSwarmBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    flatbuffers::Offset<Swarm> root) {
  fbb.FinishSizePrefixed(root);
}

#endif  // FLATBUFFERS_GENERATED_STATES_H_
