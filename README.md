# Qubic Reference Miner
This Repo contains the reference implementation of the algoritm used in Qubic.

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
Qiner <IP> <Identity> [<number of threads>]
```
- number of threads:  Optional, if not parse default number of cores will be used

# Algorithm 2025-05-15

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

Given `nonce` and `pubkey` as seeds, and constants `K`, `L`, `N`:

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