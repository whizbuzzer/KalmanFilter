*After cloning the file, go to **./build** and run `cmake --build .`
This will create the KalmanFilter executable in the **./build** folder
Then run the executable through the commandline with **./KalmanFilter**

*matplotplusplus library was installed on this system using vcpkg.

*Refer to https://github.com/alandefreitas/matplotplusplus#embed-with-cpmcmake for CMake configuration details

*If opencv package is not getting included, then move the "opencv2" folder out of the "opencv4" folder:

`sudo mv /usr/local/include/opencv4/opencv2/ /usr/local/include/`

Also, make sure that **opencv4.pc** is present in **/usr/local/lib/pkgconfig/**
If it is not, then:
1. If **opencv4.pc** exists in **/usr/lib/x86_64-linux-gnu/pkgconfig/**, then execute:

`sudo cp /usr/lib/x86_64-linux-gnu/pkgconfig/opencv4.pc  /usr/local/lib/pkgconfig/`

2. Otherwise, follow: https://forums.developer.nvidia.com/t/compiling-opencv-c-program/160402