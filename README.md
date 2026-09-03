# Qubic Reference Miner
This Repo contains the reference implementation of the algoritm used in Qubic.

## Licensing

This project is licensed under the **Anti-Military License**—see the `LICENSE` file for details.

### Third-Party Licenses

This project incorporates code from third-party sources which are governed by different licenses. Full compliance information, including the original copyright notices and terms for these dependencies, can be found in the **`NOTICE`** file in the repository root.

## File structure
- score_common.h: shared functions used for scoring (the random2 pool generator).
- Qiner.cpp: Contains the main process logic/functionality. Mainly shows how to communicate with the node and drive the Standalone miner.
- ant/: the ant-colony reference miner - `AntMiner.cpp` (tree search over the bpp9000 scorer, see the "ant colony on bpp9000" algorithm section) and `ant_colony_message.h` (wire structs, mirroring core `src/network_messages/ant_colony_message.h`).
- K12AndKeyUtill.h, keyUtils.h, keyUtils.cpp: Provide K12 and key conversion utilities/functions.

The algorithm-specific files (the scorer and its task format) are listed in the algorithm section.

# Requirement
- CPU: support at least AVX2 instruction set
- OS: Windows, Linux

# Build
## Windows
### Visual Studio 2022
- Open Qiner.sln
- Build
### Other Visual Studio versions

- Support generation using CMake with below command
```
# Assume in Qiner folder
mkdir build
cd build
"C:\Program Files\CMake\bin\cmake.exe" -G <Visual Studio Generator>
# Example: C:\Program Files\CMake\bin\cmake.exe" -G "Visual Studio 17 2022"
```
- Open Qiner.sln in build folder and build

### Enable AVX512
- Open Qiner.sln
- Right click Qiner->[C/C++]->[Code Generation]->[Enable Enhanced Instruction Set] -> [...AVX512] -> OK

## Linux
Currently support GCC and Clang
- Installed required libraries

For example,
- Ubuntu with GCC
```
sudo apt install build-essential
```
- Ubuntu with Clang
```
sudo apt install build-essential
sudo apt install clang
```


### GCC
Run below command
```
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8
```

### Clang
Run below command
```
mkdir build
cd build
CC=clang CXX=clang++ cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8
```

### Enable AVX512
To enable AVX512, -DENABLE_AVX512=1 need to be parse in the cmake command.

Example,
```
# GCC
cmake .. -DCMAKE_BUILD_TYPE=Release -DENABLE_AVX512=1

# Clang
CC=clang CXX=clang++ cmake .. -DCMAKE_BUILD_TYPE=Release -DENABLE_AVX512=1
```

# Run
```
./Qiner <Node IP> <Node Port> <MiningID> <Signing Seed> <Mining Seed> <Number of threads>
```
The active algorithm may take an extra trailing argument - see its section below.

Example: 
```
./Qiner 192.168.1.2 31841 BZBQFLLBNCXEMGLOBHUVFTLUPLVCPQUASSILFABOFFBCADQSSUPNWLZBQEXK aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa aaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa 8
```

# Algorithm 2026-08-14 (ant colony on bpp9000)

Standalone bpp9000 mining (`Qiner`) searches from your identity's root network for any network that scores at or below the epoch threshold; every solution stands alone. **Ant-colony mining is tree search over the same scorer.** You take a *parent* network already in the tree - the epoch's shared root, or a node you placed earlier - inherit its LUTs, mutate them under your nonce, and score the result. A hit must clear the epoch threshold **and** strictly beat its parent's score. On acceptance it becomes a new tree node that can be extended further, so the colony converges toward the single lowest-error network of the epoch. Lower score is better - the score is an error count.

Trees are **per-identity**: a forest, one tree per mining identity, and you only ever extend your own nodes. `AntMiner.cpp` is the reference implementation; the Standalone `Qiner` is unchanged and still available.

## Where the tree lives

The authoritative tree lives on the **node** - it is the consensus store (a per-identity forest), and every accepted solution becomes a node in it. The node also recomputes every claimed score deterministically and gates acceptance, so the miner's real job is to reproduce the node's score bit-exactly and submit on time. Everything the miner keeps locally is advisory; consensus lives on the node.

But the miner does **not** have to treat that tree as remote storage it queries on every step. Extending a parent needs the parent's LUT (its evolved ANN), and scoring is fully deterministic in `(parentLUT, pubkey, nonce, anchorDigest)` - so a miner can **build the tree locally and avoid fetching ANNs**:

- **Your own nodes** - the miner already holds the winning `bestANN` of every solution it computed, so extending your own frontier needs no fetch. The reference `AntMiner` keeps exactly that (`ownNodes[].ann`) and, since trees are per-identity, only ever extends its own nodes - so its mining loop fetches no ANN at all.
- **A node you did not compute** (a pool splitting one identity across workers, or after a restart) - either ask the node for the parent's ANN, or rebuild it locally by replaying the lineage from the root under the on-chain `(nonce, anchor)` values. Both land on the same LUT; the local rebuild trades CPU for network round-trips. `AntMiner` takes the first route once at startup (step 3b): without it a restarted miner abandons the tree it paid deposits to build and climbs again from the root.

The node still owns what only it can: the epoch context (root seed, threshold, freshness window, child cap), the anchors, acceptance gating, and the deposit refund/forfeit. Full wire and consensus contract: core `doc/ant_colony/ant_colony_miner_guide.md`.

## Build and run

`AntMiner` builds alongside `Qiner` from the same CMake / `Qiner.sln` (target `AntMiner`).

```
./AntMiner <Node IP> <Node Port> <MiningID> <Signing Seed> [Threads] --task <task file> [--operator SEED]
```
There is **no Mining Seed argument** (unlike the Standalone `Qiner`): the pool seed is the epoch spectrum digest, which `AntMiner` reads from the node's epoch context - so it cannot mine a wrong or stale pool by hand. `--task` is required and must be the epoch's pinned task file; `AntMiner` checks its topology/data hashes against the node's canonical hashes and refuses to mine on a mismatch (a wrong task wastes work and forfeits deposits).
- **Signing Seed** signs the solution `BROADCAST_MESSAGE`s. Its identity must hold enough QUs for message dissemination (`MESSAGE_DISSEMINATION_THRESHOLD`) and must not be the MiningID itself (the node treats a self-signed message as encrypted).
- **`--operator SEED`** signs the identity-tree queries (`REQUEST_ANT_IDENTITY_TREE`).

Example:
```
./AntMiner 192.168.1.2 31841 BZBQ...WLZBQEXK aaaa...aaa 8 --task task_bpp9000.bin --operator bbbb...bbb
```

## The mining loop

What `AntMiner` does each round:

1. **Epoch context** (`REQUEST_ANT_EPOCH_CONTEXT`) - spectrum digest (seeds the `random2` pool), canonical task hashes, threshold, freshness window, per-parent child cap.
2. **Task check** - load `--task`, require its topology/data hashes to equal the node's; abort otherwise.
3. **Root** - `deriveRootANN(spectrumDigest)` from the pool: the epoch's shared root network, identical for every identity. Never stored on-chain; you compute it. The spectrum digest both SEEDS the pool and is the root seed; per-identity variation enters only through the mutation seeds (`K12(publicKey || nonce || anchor)`).
3b. **Adopt the existing tree** (startup only) - page this identity's tree (`REQUEST_ANT_IDENTITY_TREE`) and fetch each node's stored ANN (`REQUEST_ANT_PARENT_ANN`), converting the canonical rows back through `updatedNeuronIndices`, so a restart resumes from the on-chain frontier instead of the root. Nothing is persisted locally; the node is the source of truth. Costs one tree walk plus one ANN fetch per node. A node whose ANN the pool evicted is kept for bookkeeping but never extended, and without `--operator` the query is refused and the miner warns it is restarting from ROOT.
4. **Parent selection** - the best resolved own node (lowest score = deepest frontier), else the root. 1-in-8 rounds explore a random resolved node or the root instead, so the search does not lock into one basin. (Pools tune this policy.)
5. **Anchor first** - pick the latest completed tick (stepping back past ticks the node stored no data for). Its digest `K12(anchorTick \|\| K12(TickData))` is part of the child RNG seed, so the anchor is fixed *before* mining and keeps the hit inside the freshness window.
6. **Search** - random canonical nonce, `computeScoreFromParent(parentLUT, pubkey, nonce, anchorDigest)`; keep a hit when `score <= threshold` and `score < parentScore`.
7. **Submit** - only if the anchor is still inside the freshness window (else drop as stale) and the parent is under its child cap. The cap is checked against the node's own `childCount` from the last tree read plus any submissions it has not caught up with yet - never against a counter local to this process, which a restart would silently reset.

   **Why the miner gates this.** Your `BROADCAST_MESSAGE` is free, but it is half the path: the computor turns an accepted message into an `AntColonyMiningSolutionTransaction` funded with `SOLUTION_SECURITY_DEPOSIT` from its own balance, refunded only on `Valid`/`ValidNotStored` with a matching claimed score. Every reject keeps the deposit. The node pre-filters duplicates and over-cap candidates, but `childCount` only grows, so a parent can reach the cap between publication and execution - a race only the miner can reduce. A miner submitting solutions the node will reject is spending its computor's money. Sent as a `BROADCAST_MESSAGE` whose decrypted `gammingKey[0] == 3` (`MESSAGE_TYPE_ANT_SOLUTION`); payload = parent ref + anchor tick + claimed score + nonce.
8. **Resolve** - every ~10 s, read your own tree back (operator-signed `REQUEST_ANT_IDENTITY_TREE`), learn each submitted node's self-ref (a child can only extend a parent whose ref is known), and read tree growth. A listing entry carries no nonce, so a submission is matched on `(score, anchorTick, depth, parentRef)` and each entry is **claimed** as it binds - otherwise two of your own hits sharing that tuple would both take the same entry. A wrong pick among interchangeable entries is caught by the LUT self-check in step 9. A node still not on-chain after 3 resolve cycles warns of a likely **anchor-digest mismatch** - compare the printed `Anchor tick N digest=` with the node's F3 line.
9. **LUT self-check** - every 60 s, fetch each resolved own node's stored ANN back once (operator-signed `REQUEST_ANT_PARENT_ANN`) and compare it with the local copy; a node is checked only once, so the cost is one fetch per accepted solution. On mismatch the node is excluded from parent selection - children mined from a wrong local LUT would score differently on-chain, wasting work and forfeiting deposits.

**ANN layout note.** The node's canonical ANN bytes store the LUT rows of the **non-input neurons in ascending index, densely** (row `k` = neuron `updatedNeuronIndices[k]`, tail rows zero). This miner stores rows by absolute neuron index, so ANN bytes exchanged with a node must be converted through `updatedNeuronIndices` - never compared directly. Full byte map: core `doc/ant_colony_mining.md`, section 2.7(c).

## Canonical ant nonce

`computeScoreFromParent` rejects a non-canonical nonce with `INVALID_SCORE_VALUE` (`0xFFFFFFFF`), and so does the node - a rejected score forfeits the deposit, so the miner canonicalizes every nonce before scoring.

| Byte | Meaning | Range |
|---|---|---|
| `nonce[0]` | algorithm select | `1` (bpp9000) |
| `nonce[1]` | `L` - LUT entries changed per mutation step | `[1, 10]` (`MAX_LUT_ENTRIES_PER_STEP`) |
| `nonce[2]` | `K` - explore-phase length (anti-attractor) | `[0, 100]` (`NUMBER_OF_MUTATIONS`) |
| `nonce[3..31]` | search-path seed | any |

Contrast with the Standalone `Qiner`, which *clamps* `L` and forces `K = 0`; the ant path requires the values already in range.

## Scorer API - two entry points, one walk

Both entry points feed the same anti-attractor local search (`computeScoreFromCurrent(L, K, cur)`); they differ only in where the starting LUTs and the mutation seeds come from.

| Aspect | Standalone (`Qiner`) | Ant (`AntMiner`) |
|---|---|---|
| Entry | `computeScore(pubkey, nonce)` | `deriveRootANN` + `computeScoreFromParent(parentLUT, pubkey, nonce, anchorDigest)` |
| Starting LUTs | `random2(K12(pubkey))` - your root | the parent's LUT (root or a fetched node) |
| Mutation seeds | `K12(pubkey \|\| nonce[3..31])` | `K12(pubkey \|\| nonce[3..31] \|\| anchorDigest)` - anchor-bound |
| `K` (explore) | forced 0 | `nonce[2]` |
| Pool | owned (`initialize` fills it) | shared read-only (`setPool`), one pool across all threads |

`bestANN` (via `getBestANN`) is the evolved network of the winning search; it becomes the child node's LUT that the next depth extends.

# Algorithm 2026-07-16 (bpp9000)

## Files
- score_bpp9000.h: the bpp9000 scorer (a recurrent ternary-LUT network).
- task_file.h: the unified task-file format (topology + data blocks) with pack/parse and hash helpers.

## Run argument
The miner takes one optional trailing CLI argument - `[Task file path]` - the path to the bpp9000 task file; it defaults to `task_bpp9000.bin`.

## Overview
A recurrent ternary-LUT network scored over a windowed sequence. Given a task (a fixed network wiring plus a sequence of input/output samples), mining searches the per-neuron lookup tables (LUTs) for the configuration that reproduces the samples with the fewest errors. The score is that error count (lower is better). The task is loaded from a unified task file (defaults to `task_bpp9000.bin`).

## Key concepts
- Every value is a **trit** in `{0, 1, 2}`, where `2 = UNKNOWN`.
- The network has `P` neurons, each typed **input** / **output** / **evolution**; each neuron owns a `27`-entry **LUT** (`3^3`).
- A neuron's next value is `LUT[t0 + 3*t1 + 9*t2]`, indexed by its three neighbours' trits.
- Wiring is explicit per neuron (`neighborIndices[n*K + k]`, any neuron in `[0, P)`). One **signal neuron** self-clocks the input feeding.
- Only the LUTs change under mutation; the wiring and the sample data are fixed by the task.

## Random sources
Where every part of the network comes from. `random2` never invents bytes - it reads them out of the pool, and the "derived from" value only selects the read positions.

| Source | Derived from | When |
|---|---|---|
| `random2` pool | epoch spectrum digest | epoch start |
| Neuron placement (input / output / signal) | read from the task file | epoch start |
| Wiring (each neuron's 3 neighbours) | read from the task file | epoch start |
| Initial LUT contents | `K12(publicKey)` | per public key |
| Mutation seeds | `K12(publicKey \|\| nonce[3..31])` | per nonce |
| Algorithm select | `(nonce[0] & 1) != 0` -> bpp9000 (odd nonce; the miner mines odd nonces) | per nonce |
| L (LUT entries changed per mutation) | `nonce[1]` | per nonce |
| K (anti-attractor length) | `nonce[2]` | per nonce |

The random pool is built from the epoch spectrum digest, in Qiner it is seen as miningSeed, the public key decide the starting LUT ,the public key and nonce decide *where each draw reads from it*. Neuron placement and wiring are no longer random: they are read from the task file.

## Constants
```
K = NUMBER_OF_NEIGHBORS        // 3, hardcoded (LUT index is base-3 over 3 trits)
N = NUMBER_OF_INPUT_NEURONS
P = POPULATION_THRESHOLD       // total neurons, power of 2
T = SEQUENCE_LENGTH            // samples in the task
W = WINDOW_WIDTH               // samples fed per window
numberOfWindows = T - W        // graded windows per score()
maxTicks = MAX_NUMBER_OF_TICKS // per-window inference budget; exceeding it fails the process
S = NUMBER_OF_MUTATIONS        // search steps
```

## Code flow (pseudocode)
```
initialize(miningSeed, taskFile):
    generateRandom2Pool(miningSeed) -> pool
    loadTaskData(taskFile):
        parse + hash-verify the topology block (input / output / signal indices + neighbour wiring)
        parse + hash-verify the data block (packed input / output trit samples)
        derive neuron roles (input / output / evolution)

// The miner tries random odd nonces until one solves:
findSolution(publicKey, nonce):
    return computeScore(publicKey, nonce) <= SOLUTION_THRESHOLD

computeScore(publicKey, nonce):             // anti-attractor local search
    L     = clamp(nonce[1], 1, MAX_L)       // LUT flips applied per step
    Kexpl = clamp(nonce[2], 0, S)           // length of the explore phase
    cur   = initializeANN(publicKey, nonce)
    best  = cur
    for s in 0 .. S-1:
        save previous LUTs
        for i in 0 .. L-1:                       // apply L LUT flips this step
            mutate(mutationSeed[s * MAX_L + i])  // each flips one LUT entry
        r = score()
        accept = (s < Kexpl) ? (r >= cur)   // explore: allow equal-or-worse
                             : (r <= cur)   // exploit: keep equal-or-better
        if accept: cur = r  else: rollback to previous LUTs
        if cur < best: best = cur
    return best

initializeANN(publicKey, nonce):
    initial LUT    = random2(K12(publicKey), pool)                 // per computor, fixed
    mutation seeds = random2(K12(publicKey || nonce[3..31]), pool) // the search path (nonce[0..2] excluded)
    set neuron types + values (UNKNOWN); set current LUTs from the initial LUT
    return score()

score():                                    // windowed, self-clocked; lower is better
    failures = 0
    for window in 0 .. numberOfWindows-1:
        reset all neurons to UNKNOWN
        feed W input samples, paced by the signal neuron, then read the settled output
        if it does not settle within maxTicks: return INFINITE_ERROR   // fails the whole network
        if predicted output != expected output: failures++
    return failures

processTick():                              // one inference step
    for each non-input neuron:
        next value = LUT[t0 + 3*t1 + 9*t2] from its three neighbours
    commit the new value into every non-input neuron

mutate(seed):                               // one LUT flip
    pick one LUT entry of one non-input neuron; set it to a different trit
```

## Task file
Three parts: `[ header ][ topology block ][ data block ]`.

- **Header** - dimensions (`N, M, T, P, K`) plus a hash of each block; lets the miner confirm it loaded the intended task.
- **Topology block** - the fixed ANN wiring: which neurons are input / output / signal, and each neuron's neighbours. Defines the network structure and never changes during mining.
- **Data block** - the sample sequence the ANN is scored against: the input/output rows it must predict across each window.

Both blocks are KangarooTwelve-hashed against the header and rejected on mismatch. Byte-level layout is in `task_file.h`.

# Previous algorithms (history)

The algorithms below are retired - their code has been removed from the miner. They are kept here for historical reference.

# Algorithm 2025-05-15 (hyperidentity)

## Definitions and precondition
- The `random2` generator will be used consistently across the entire pipeline.
- Each neuron can hold a value of `-1`, `0`, or `1`.
- Synapse weights range within the continuous interval \([-1, 1]\).
- Every neuron has exactly `2M` **outgoing** synapses. Synapses with zero weight represent *no connection*.
- A synapse is considered to be **owned** by the neuron from which it originates.
- A **mining seed** is used to initialize the random values of both input and output neurons.
- The **nonce** and **public key** determine:
  - The random placement of input and output neurons on a ring,
  - The weights of synapses,
  - The method for selecting synapses during the **evolution** step.
- Symbols,
  - S: evolution step
  - P: max neurons population
  - R: the number of mismatch between expected output and computed ouput
## I. ANN Structure Initialization

Given `nonce` and `pubkey` as seeds, and constants `K`, `L`, `N`, `2M`:

1. Initialize `K + L` neurons arranged in a ring structure.
   - `K` input neurons and `L` output neurons are placed at random positions on the ring.
2. Initialize input and output neuron values randomly.
3. Initialize weights of `2M` synapses with random values in the range `[-1, 1]` (i.e., `-1`, `0`, or `1`).
4. Convert neuron values to **trits**:
   - Keep `1` as is.
   - Change `0` to `-1`.
   - This step occurs **only once**.
5. Run initial tick simulation to initialize the `R` value.

---

## II. Tick Simulation

1. For each neuron, compute the new value as: `new_value = sum(weight × connected_neuron_value)`
2. Clamp each neuron's value to the range `[-1, 1]`.
3. Stop the tick simulation if **any** of the following conditions are met:
- All output neurons have non-zero values.
- `N` ticks have passed.
- No neuron values change.

---

## III. Evolution and Simulation

1. Compute the initial `R_best` — the number of non-matching output bits.
2. Repeat the following mutation steps up to `S` times:
    - Randomly pick a synapse and change its weight:
      - Increase or decrease it by `1` (i.e., ±1).
      - If the new weight is within `[-1, 1]`, proceed.
      - If the new weight becomes `-2` or `2`:
        - Revert the weight to its original value.
        - Insert a **new neuron** immediately after the connected neuron.
        - The new neuron:
          - Copies all **incoming** synapses from the original neuron.
          - Copies only the **mutated** outgoing synapse; all others are set to `0`.
        - Remove any synapses exceeding the `2M` limit per neuron.
3. Remove any neurons (except input/output) that:
    - Have all zero **incoming** synapses, or
    - Have all zero **outgoing** synapses.
4. Stop the evolution if the number of neurons reaches the population limit `P`.
5. Run **Tick Simulation** again.
6. Compute the new `R` value:
    - If `R > R_best`, discard the mutation.
    - If `R ≤ R_best`, accept the mutation and update `R_best = R`.

# Algorithm 2025-12-10 (addition)

## Overview
Focused on implementing an Addition function. The core changes involve the training data set size, input/output representation, and the scoring mechanism.

## Key Changes from Original Algorithm

| Aspect | Original | New |
| -----  | -------- |-----|
| Input neurons | Random, value can be changed in tick simulation | Load from training data, value unchanged in tick simulation|
| Tick simulation | Run once per inference | Run 2^Input pairs|
| Score | Matching bits for 1 pattern | Total matching bits across ALL training pairs|
| Neighbor count | Fixed (always maxNeighbors) | Dynamic (min(maxNeighbors, population-1))|

## Pseudo code
```
// ========== CONSTANTS ==========
// Can be adjusted
K = NUMBER_OF_INPUT_NEURONS      // 14 (7 bits for A + 7 bits for B)
L = NUMBER_OF_OUTPUT_NEURONS     // 8 (8 bits for result C)
N = NUMBER_OF_TICKS              // 120
M = MAX_NEIGHBOR_NEURONS / 2     // 364 (half of 728)
S = NUMBER_OF_MUTATIONS          // 100
P = POPULATION_THRESHOLD         // K + L + S = 122

TRAINING_SET_SIZE = 2^K          // 16,384
MAX_SCORE = TRAINING_SET_SIZE × L  // 131,072
SOLUTION_THRESHOLD = MAX_SCORE × 4/5  // 104,857

// ========== I. NEW DATA STRUCTURES ==========
STRUCT Pair:
    char input[K]                // K/2 bits of A, K/2 bits of B (values: -1 or +1)
    char output[L]               // L bits of C (values: -1 or +1)

Pair allPairs[ALL_PAIRS_SIZE]    // All possible (A, B, C) combinations
Pair selected[SELECTED_SIZE]     // Randomly selected training pairs

// ========== II. INITIALIZATION ==========
FUNCTION initialize(publicKey, nonce):
    // 1. Generate random2 pool
    hash = KangarooTwelve(publicKey || nonce)
    initValue = Random2(hash)

    // 2. Generate all 2^K possible (A, B, C) pairs
    boundValue = 2^(K/2) / 2     // 64 for 7-bit signed [-64, 63]
    index = 0
    FOR A = -boundValue TO boundValue-1:
        FOR B = -boundValue TO boundValue-1:
            C = A + B            // C in range [-128, 126]
            allPairs[index].input[0..K/2-1] = toTernaryBits(A, K/2)   // 7 bits
            allPairs[index].input[K/2..K-1] = toTernaryBits(B, K/2)   // 7 bits
            allPairs[index].output = toTernaryBits(C, L)              // 8 bits
            index++
    
    // 3. Initialize ANN structure
    population = K + L
    // Randomize location of input neurons and output neurons
    randomizeNeuronTypes(initValue)  // K inputs, L outputs
    // Random weights of synapses
    initializeSynapseWeights(initValue)

    // 4. Fist inference for init best score
    inferANN()

// ========== III. SCORING ==========
FUNCTION inferANN():
    totalScore = 0

    // Evaluate ANN on all 2^K pairs
    FOR i = 0 TO ALL_PAIRS-1:
        // Load input values (these stay CONSTANT during ticks)
        setInputNeurons(selected[i].input)

        // Reset output neurons to 0
        resetOutputNeurons()

        // Run tick simulation
        runTickSimulation()

        // Count matching output bits
        FOR j = 0 TO L-1:
            IF outputNeuron[j].value == selected[i].output[j]:
                totalScore++

    RETURN totalScore

// ========== IV. TICK SIMULATION ==========
// Same as before, but:
// - Runs on K input neurons and L output neurons
// - Input neuron values are PRESERVED (not updated during ticks)
// - Uses dynamic neighbor count: min(MAX_NEIGHBOR_NEURONS, population - 1)

FUNCTION runTickSimulation():
    FOR tick = 0 TO N-1:
        // Calculate weighted sums for all neurons
        actualNeighbors = min(MAX_NEIGHBOR_NEURONS, population - 1)
        FOR each neuron n in population:
            sum = 0
            FOR each neighbor m within actualNeighbors:
                sum += neurons[m].value × synapses[n→m].weight
            neuronValueBuffer[n] = sum

        // Update only NON-INPUT neurons
        FOR each neuron n in population:
            IF neurons[n].type != INPUT:
                neurons[n].value = clamp(neuronValueBuffer[n], -1, +1)

        // Early exit conditions
        IF allNeuronsUnchanged() OR allOutputsNonZero():
            BREAK

// ========== V. MUTATION ==========
// Same as before
FUNCTION mutate(step):
    actualNeighbors = min(MAX_NEIGHBOR_NEURONS, population - 1)
    synapseIdx = random(initValue.synapseMutation[step]) % (population × actualNeighbors)

    IF currentWeight + mutation is valid (-1, 0, +1):
        synapse[synapseIdx].weight += mutation
    ELSE:
        // Weight overflow → INSERT new neuron
        insertNeuron(synapseIdx)
        population++

    // Remove redundant neurons (all-zero incoming OR outgoing synapses)
    WHILE hasRedundantNeurons():
        removeRedundantNeurons()

// ========== MAIN LOOP ==========
FUNCTION computeScore(publicKey, nonce):
    bestScore = initialize(publicKey, nonce)
    bestANN = copy(currentANN)

    FOR s = 0 TO S-1:
        mutate(s)

        IF population >= P:
            BREAK

        newScore = inferANN()

        IF newScore > bestScore:
            bestScore = newScore
            bestANN = copy(currentANN)
        ELSE:
            currentANN = copy(bestANN)  // Rollback

    RETURN bestScore
```
