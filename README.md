# libfranka examples

## Setup instructions

### Clone the libfranka repo
```
git clone --recursive https://github.com/KhachDavid/libfranka.git
cd libfranka
# branch “panda” is already checked out; if not, do:
git checkout panda
```

### Configure and Build
```
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DBUILD_TESTS=OFF        \
         -DBUILD_EXAMPLES=ON
cmake --build . -j$(nproc)
```

### Install to the local prefix
```
cmake --install . --prefix $HOME/panda_libfranka/install
```

### Clone this repository

```
export FRANKA_INSTALL=<PATH_TO_LIBFRANKA>/libfranka/install
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j 32
```

### Move binary to franka

Make sure franka FCI is activated

```
scp move_joint7_position student@station:~/your_path
```

When executing, franka should move joint 7 and provide a very small error, which should be around 1/5th of one degree.
