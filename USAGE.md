
## Getting Started
- Download and compile `OpenCV 4.1.1` from https://opencv.org/
- Download and compile `Dlib 19.17` from http://dlib.net/
- Clone this repo:

```bash
git clone https://github.com/NeuralVFX/realtime-head-pose-open-cv.git
```

## Setting Up Visual Studio

#### VC++ Directories
- Replace the `Dlib` and `OpenCV` paths under `Library Directories` and `Include Directories`
#### C/C++ 
- Replace the `Dlib` and `OpenCV` paths under `Additional Include Directories`
#### Linker
- Replace `OpenCV` path under `General / Additional Libraries Directories`
- Replace `Dlib` and `OpenCV` `.lib` files and under `Input / Additional Dependencies`

## Compile
- Compile in release mode
- `head-pose-opencv.dll` will be created in the `\x64\Release` directory

## Use
- `head-pose-opencv.dll` and OpenCV and Dlib `.dll` files must be added to Unity Plugins directory
- SSD and Landmark Detection models must be downloaded and placed into the Unity project directory
- Detailed instuction for using with Unity, and an example project using `head-pose-opencv.dll` can be found here:  https://github.com/NeuralVFX/unity-head-pose-example

