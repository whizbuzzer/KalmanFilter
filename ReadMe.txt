*After cloning the file, go to **./build**
Then if **./build** is empty, then run `cmake ..` and run `cmake --build .`
Otherwise, just `cmake --build`
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


* Make sure that you have all the available codecs installed for running video files and that you have built and installed opencv ONLY AFTER installing said codecs:

1. `sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev`

2. Then go to the directory where you have opencv and opencv-contrib unzipped and create a build folder

3. Go inside this build directory and open terminal

4. In terminal, execute:
`cmake -DFFMPEG=1 -DWITH_OPENCL=1 -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.x/modules ../opencv-4.x`

5. Build the opencv install file with:
` cmake --build .`

6. Install opencv with:
`sudo make install`


* All data files should be put in the /data folder and provide the relative path to them in functions as **"../data/filename.type"**


* ".h" files are commonly associated with C files. 
- Either declare classes and functions in ".h" and define them in corresponding ".cpp" files OR define as well as declare them in ".hpp" files which are associated with C++

