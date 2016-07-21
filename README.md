# RobotSimulator
VREP simulation repo

## Build

```bash
export VREP_HOME=/home/user/vrep/...

mkdir build
cd build
cmake ..
make
```

## Run
   
```bash
cd build
./robot PORT LeftMotor RoghtMotor
```

## Run tests and code coverage

```bash
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
ctest --verbose
make test_coverage
```

After successful run coverage results are placed in `build/coverage/index.html`.
