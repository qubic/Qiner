#pragma once

// Ant-colony wire protocol: the one definition shared by the miner and the operator tools.
// The client-side subset of core/src/network_messages/ant_colony_message.h, which is
// authoritative; differs only in m256i (here unsigned char[32]) and the type() helpers.
// The static_asserts catch drift.

// Types from core/src/network_messages/network_message_type.h; BROADCAST_MESSAGE is in network.h.
#define MESSAGE_TYPE_ANT_SOLUTION 3   // the decrypted gammingKey[0], not a network message type
#define BROADCAST_FUTURE_TICK_DATA 8
#define REQUEST_TICK_DATA 16
#define REQUEST_CURRENT_TICK_INFO 27
#define RESPOND_CURRENT_TICK_INFO 28
#define END_RESPONSE 35
#define REQUEST_ANT_IDENTITY_TREE 72
#define RESPOND_ANT_IDENTITY_TREE 73
#define REQUEST_ANT_PARENT_ANN 74
#define RESPOND_ANT_PARENT_ANN 75
#define REQUEST_ANT_EPOCH_CONTEXT 76
#define RESPOND_ANT_EPOCH_CONTEXT 77

// ROOT sentinel of a parent reference (matches core SolutionRef ROOT_REF).
static constexpr unsigned int ROOT_TICK = 0U;
static constexpr unsigned int ROOT_INDEX_IN_TICK = 0xFFFFFFFFU;

static constexpr unsigned int SIGNATURE_SIZE = 64;

struct RespondCurrentTickInfo
{
    unsigned short tickDuration;
    unsigned short epoch;
    unsigned int tick;
    unsigned short numberOfAlignedVotes;
    unsigned short numberOfMisalignedVotes;
    unsigned int initialTick;
};
static_assert(sizeof(RespondCurrentTickInfo) == 16, "RespondCurrentTickInfo unexpected size");

struct RequestedTickData
{
    unsigned int tick;
};
static_assert(sizeof(RequestedTickData) == 4, "RequestedTickData unexpected size");

// sizeof(TickData) in core: 8 + 8 + 32 + NUMBER_OF_TRANSACTIONS_PER_TICK(4096) * 32
//                           + MAX_NUMBER_OF_CONTRACTS(1024) * 8 + SIGNATURE_SIZE(64).
#define TICK_DATA_SIZE 139376U

// The parents one identity can branch from; a child must name a parent in its OWN tree.
// Operator-signed: SIGNATURE_SIZE bytes follow the payload. Paginated via fromIndex / nextIndex.
struct RequestAntIdentityTree
{
    unsigned char pubkey[32];   // whose tree to report; the miner's own identity
    unsigned int fromIndex;     // resume cursor; 0 on the first call
    unsigned int padding;
};
static_assert(sizeof(RequestAntIdentityTree) == 40, "RequestAntIdentityTree unexpected size");

// Followed by count * AntIdentityTreeNode; itemSize lets the receiver validate without
// hardcoding the entry size.
struct RespondAntIdentityTreeHeader
{
    unsigned int count;
    unsigned int itemSize;
    unsigned int nextIndex;     // resume cursor for the next request; 0 means no more records
};
static_assert(sizeof(RespondAntIdentityTreeHeader) == 12, "RespondAntIdentityTreeHeader unexpected size");

// One stored node of the identity's tree. score is an error count - a child must score strictly
// below it. No nonce or identity field, so a miner cannot match an entry to its own submission
// exactly - see the claim/consume matching in AntMiner.cpp.
// childCount is counted only up to the cap, so it reads 0 for every entry while the cap is unbound.
struct AntIdentityTreeNode
{
    unsigned int selfTick;            // a child sets this as its parentRef
    unsigned int selfSolutionIndexInTick;
    unsigned int parentTick;          // this node's own parent; (0, 0xFFFFFFFF) = root
    unsigned int parentSolutionIndexInTick;
    unsigned int score;
    unsigned int childCount;
    unsigned int anchorTick;
    unsigned int depth;
};
static_assert(sizeof(AntIdentityTreeNode) == 32, "AntIdentityTreeNode unexpected size");

// One node's stored network, named by parentRef - the ANN a miner mutates to extend it.
// Operator-signed, like the identity-tree request.
struct RequestAntParentAnn
{
    unsigned int parentRefTick;
    unsigned int parentRefSolutionIndexInTick;
};
static_assert(sizeof(RequestAntParentAnn) == 8, "RequestAntParentAnn unexpected size");

// RespondAntParentAnnHeader.status values.
static constexpr unsigned char ANT_PARENT_ANN_STATUS_OK = 0;        // ANN bytes follow the header
static constexpr unsigned char ANT_PARENT_ANN_STATUS_NOT_FOUND = 1; // parentRef has no record
static constexpr unsigned char ANT_PARENT_ANN_STATUS_IS_ROOT = 2;   // ROOT_REF; no ANN payload - miner derives the shared epoch root

// On status Ok, annSizeBytes bytes of canonical ANN follow - one trit per byte, the form the
// scorer consumes. 0 for every other status.
struct RespondAntParentAnnHeader
{
    unsigned int parentRefTick;
    unsigned int parentRefSolutionIndexInTick;
    unsigned int annSizeBytes;
    unsigned char status;
    unsigned char padding[3];
};
static_assert(sizeof(RespondAntParentAnnHeader) == 16, "RespondAntParentAnnHeader unexpected size");

// Per-epoch parameters a miner needs to start building solutions. The anchor digest is not here;
// derive it from the anchor tick's TickData (REQUEST_TICK_DATA): transactionDigest = K12(TickData),
// then K12(anchorTick || transactionDigest). Packed to mirror core.
#pragma pack(push, 1)
struct RespondAntEpochContext
{
    unsigned char spectrumDigest[32];   // the shared root seed; SEEDS the random2 pool, root = deriveRootANN(spectrumDigest)
    unsigned char topologyHash[32];     // canonical task topology-block hash (BPP9000_TOPOLOGY_HASH)
    unsigned char dataHash[32];         // canonical task data-block hash (BPP9000_DATA_HASH)
    unsigned int threshold;             // score threshold for this epoch (lowered on the test node)
    unsigned int freshnessWindow;       // N: publish within N ticks of the anchor
    unsigned int solutionCount;         // accepted solutions so far (the tree-growth readout)
    unsigned int freeAnnSlotsCount;
    unsigned int maxChildrenPerParent;  // ANT_MAX_CHILDREN_PER_PARENT; 0 = unbound
    unsigned short epoch;
    unsigned short padding;
};
#pragma pack(pop)
static_assert(sizeof(RespondAntEpochContext) == 120, "RespondAntEpochContext unexpected size");

// The payload following a BroadcastMessage(MESSAGE_TYPE_ANT_SOLUTION) header.
struct AntSolutionBroadcastPayload
{
    unsigned int parentTick;            // ABSOLUTE
    unsigned int parentSolutionIndexInTick;
    unsigned int anchorTick;            // ABSOLUTE
    unsigned int claimedScore;
    unsigned char nonce[32];
};
static_assert(sizeof(AntSolutionBroadcastPayload) == 48, "AntSolutionBroadcastPayload unexpected size");
