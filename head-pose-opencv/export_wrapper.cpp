#pragma once
#include "pose_estimate.h"
#include <iostream>
#include <stdio.h>
#include <cstdio>



Estimator PoseEstimator = Estimator();


extern "C" int __declspec(dllexport) __stdcall  Init(int& outCameraWidth, int& outCameraHeight, int detectRatio, int camId)
{
	return PoseEstimator.init(outCameraWidth, outCameraHeight, detectRatio, camId);
}


extern "C" void __declspec(dllexport) __stdcall  Close()
{
	PoseEstimator.close();
}


extern "C" void __declspec(dllexport) __stdcall Detect(TransformData& outFaces)
{
	PoseEstimator.detect(outFaces);
}


extern "C" void __declspec(dllexport) __stdcall GetRawImageBytes(unsigned char* data, int width, int height)
{
	PoseEstimator.getRawImageBytes(data, width, height);
}

