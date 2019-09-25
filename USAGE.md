
## Getting Started
- Install Visual Studio 2017
- Download and compile `OpenCV 4.1.1` from https://opencv.org/
- Download and compile `Dlib 19.17` from http://dlib.net/

- Clone this repo:

```bash
git clone https://github.com/NeuralVFX/realtime-head-pose-open-cv.git
```

## Setting Up Visual Studio

#### VC++ Directories
- Replace the `Dlib` paths under `Library Directories` and `Include Directories`
#### C/C++ 
- Replace the `Dlib` and `OpenCV` paths under `Additional Include Directories`
#### Linker
- Replace `OpenCV` path under `General / Additional Libraries Directories`
- Replace `Dlib` and `OpenCV` `.lib` files and under `Input / Additional Dependencies`
- If using `OpenVino` version of `OpenCV`, you can remove my `OpenCV` `.lib` files and only use `opencv_world401.lib` under `Input / Additional Dependencies`

## Compile
- Compile in release mode
- `head-pose-opencv.dll` will be created in the `\x64\Release` directory

## Use
- Detailed instuction for using with Unity, and an example project using `head-pose-opencv.dll` can be found here:  https://github.com/NeuralVFX/unity-head-pose-example

