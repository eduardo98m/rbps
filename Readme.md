# Rigid Body Physics Simulation (RBPS)
![Tests](https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/eduardo98m/1a5fc2bac537cce52c590a74d4d192e3/raw/tests.json)
[![Coverage](https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/eduardo98m/1a5fc2bac537cce52c590a74d4d192e3/raw/coverage.json)](https://eduardo98m.github.io/rbps/)

**RBPS** is a physics engine...

# Requisites

* Currently we use coal for collision detection and response (GJK + EPA)

To install it execute the following commands:

```sh
sudo apt install libeigen3-dev libboost-serialization-dev libboost-filesystem-dev libassimp-dev liboctomap-dev libqhull-dev
```

Download the library

```sh
wget https://github.com/coal-library/coal/releases/download/v3.0.2/coal-3.0.2.tar.gz
```

Extract the library

```sh
tar -xzf coal-3.0.2.tar.gz
```

```sh
mkdir build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCOAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL=OFF \
  -DBUILD_PYTHON_INTERFACE=OFF \
  -DCOAL_HAS_OCTOMAP=OFF \
  -DCOAL_HAS_QHULL=ON
```