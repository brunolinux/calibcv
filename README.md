# calibcv
some tests using opencv to make a camera calibration pipeline

## to compile

### Cloning

This repo depends on some other repos I was making. It now depends on [**felixcam**](https://github.com/wpumacay/felixcam), which is a repo that I'm making for a general API to handle different types of cameras in different environments.

Just make sure you clone with **--recursive** to grab that dependency.

    git clone --recursive https://github.com/wpumacay/calibcv.git

### Windows build

Just make sure you have the opencv windows-pack extracted in **C:/opencvbuild**. Check the flags **OpenCV_DIR** in the root [**CMakeLists.txt**](CMakeLists.txt) file.

Also, there seems to be an issue with opencv in windows when building in debug mode in windows 10. The pipeline runs 10 to 20 times slower than the linux build. Just build in **Release** mode until I figure out what is going on in the opencv build. Maybe they forgot to uncomment some logs or some silent debug functionality in the windows source code.

Configure the project with cmake-gui using the vc15 - x64 compiler. Make sure you generate for the compilers you have available. Also, for now just **MAKE SURE** you make an inplace build ( build directory == source directory, in the cmake-gui tool ), as some resources are relative to the source path.

Then, just build the project using the generated solution file.

### Linux build

Life is good when using linux :D ( well, most of the time ). Just run :

    cmake .
    make

And voila, you already have everything built.

**Note** : I've been playing with the build steps to make the project build in windows. Please, if you find something wrong with the linux build it might be that I misconfigured some steps. Just let me know and I'll try to fix it as quickly as I can.

### the resources

To get the calibration videos just do :
    
    cd res
    sh get_calib_files.sh

This will download the calibration videos from dropbox. Make sure you have them, as some examples depend on them. If you are in windows, just grab the dropbox links from the script and download the files manually.

### Samples

* testPipelineVideo : this is an example showing the pipeline in the ps3eyecam calibration video.

* testPipeline : this example uses the pipeline in a live-strean from a camera. This uses the base videohandler ( [**SVideoHandler.h**](engine/src/SVideoHandler.cpp) ) which wraps the functionality of the cv::VideoCapture and adds some extra functionality.

* testPipelinePS3 : The same as the previous, but it uses the PS3 video handler ( [**SVideoHandlerPSEye.cpp**](engine/src/SVideoHandlerPSEye.cpp) ) which wraps the v4l2 driver in linux to handle the camera at 60fps.