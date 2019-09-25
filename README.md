# Realtime-Head-Pose-Open-CV

This project is a C++ ( DLL) implimentation of realtime headpose tracking using OpenCV and DLIB.

## Interesting Stuff

- The output of the Visual Studio Project is a `DLL`, which can be added to a Unity Project and accessed through C#
- Uses `cv::dnn` to ro run an `SSD face detection` on the scene
- Feeds the most confident result to `dlib::shape_predictor` to detect 68 landmarks
- Also contains a function to return the pixels from the video feed

## Estimation Pipeline Example
![](examples/pose_pipeline_example.png)


## Code Usage
Usage instructions found here: [user manual page](USAGE.md).




