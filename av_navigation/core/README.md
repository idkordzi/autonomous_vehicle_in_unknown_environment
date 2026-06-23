# Autonomous drone tracking objects in urban environment

## Project scope

This is implementation of core functionality of software for autonomous drone whose puprose is to track and follow chosen moving object.

## Code build

### Requirements

- C/C++ compilers (gcc/g++)
- Python (>3.10)
- CUDA (>13.0)
- CMake

### Prerequirements

This application uses YOLO model for object detection.

Download model by executing script:

```
python3 python/drone_vision/download_yolo.py
```

You can configure the model by editing the script.

### Code build

Build code with CMake:

```
cmake -S . -B build/  # cmake project configuration
cmake --build build/  # build code
```

## Code test

Test script are located in `test/` directory, organized by module.

You can change tests by editing the files.

Tests are run by executing appropriate file, ex.:

```
./build/drone_vision_test  # run tests for vision module
```

You can also use VSCode launch file and run tests via editor.
