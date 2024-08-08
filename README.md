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

