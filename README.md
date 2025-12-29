# Discrete Kalman Filter for 1D and 2D object tracking


---
## Description

This project aims to implement the Kalman Filter (Linear Quadratic Estimation) for Object-tracking in 1D and 2D in C++ using [this reference](https://machinelearningspace.com/object-tracking-python/)

Through this port, I got a brush up on using CMake and gained exposure to an amazing graph plotting library for C++, [Matplot++](https://github.com/alandefreitas/matplotplusplus) along with the [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) C++ library used for linear algebra and the [OpenCV](https://opencv.org/) C++ library used for image processing and computer vision techniques.

### System Information
Type | Info
---|---
Operating System | Ubuntu 22.04
Processor | AMD® Ryzen 9 5900hx with radeon graphics × 16
Graphics card | NVIDIA Corporation GA106M [GeForce RTX 3060 Mobile / Max-Q] / NVIDIA GeForce RTX 3060 Laptop GPU/PCIe/SSE2
Installed RAM | 16.0 GB (15.7 GB usable)
System type | 64-bit operating system, x64-based processor
Pen and touch | No pen or touch input is available for this display


---
## Installation

### Prerequisites
- Ubuntu 22.04
- CMake ≥ 3.20
- C++17-compatible compiler
- OpenCV (with video codec support)
- Matplot++ (installed via vcpkg)

### Project structure:
project_root/
├── build/
├── data/
├── include/
└── src/

### Installation Steps

1. After cloning the repository, create a `build` directory with `mkdir build` and then go inside the build directory with `cd build`

2. Run `cmake ..` from inside the `/build` directory (which you already should be in) if you are building for the first time.\
This step just tells CMake to look for `CMakeLists.txt` in the `/project_root` directory.

3. Run `cmake --build .` from inside the build directory.\
This will create the KalmanFilter executable in the **./build** folder
Then run the executable through the commandline with **./KalmanFilter**


**matplotplusplus library was installed on this system using vcpkg.**


**Refer to https://github.com/alandefreitas/matplotplusplus#embed-with-cpmcmake for CMake configuration details**


If opencv package is not getting included, then move the "opencv2" folder out of the "opencv4" folder:

`sudo mv /usr/local/include/opencv4/opencv2/ /usr/local/include/`

Also, make sure that **opencv4.pc** is present in **/usr/local/lib/pkgconfig/**\
If it is not, then:
1. If **opencv4.pc** exists in **/usr/lib/x86_64-linux-gnu/pkgconfig/**, then execute:

`sudo cp /usr/lib/x86_64-linux-gnu/pkgconfig/opencv4.pc  /usr/local/lib/pkgconfig/`

2. Otherwise, follow: https://forums.developer.nvidia.com/t/compiling-opencv-c-program/160402


Make sure that you have all the available codecs installed for running video files and that you have built and installed opencv ONLY AFTER installing said codecs:

1. `sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev`

2. Then go to the directory where you have opencv and opencv-contrib unzipped and create a build folder

3. Go inside this build directory and open terminal

4. In terminal, execute:
`cmake -DFFMPEG=1 -DWITH_OPENCL=1 -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.x/modules ../opencv-4.x`

1. Build the opencv install file with:
` cmake --build .`

1. Install opencv with:
`sudo make install`


* All data files should be put in the /data folder and provide the relative path to them in functions as **"../data/filename.type"**


* ".h" files are commonly associated with C files. 
- Either declare classes and functions in ".h" and define them in corresponding ".cpp" files OR define as well as declare them in ".hpp" files which are associated with C++

