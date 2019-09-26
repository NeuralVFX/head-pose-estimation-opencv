#pragma once
#include <iostream>
#include <stdio.h>
#include <cstdio>

#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/dnn.hpp"
#include "opencv2/dnn/shape_utils.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "dlib/image_processing.h"
#include "dlib/opencv.h"
#include "dlib/image_processing/frontal_face_detector.h"


using namespace std;


// Struct to pass data from DLL
struct TransformData
{
	TransformData(float tx, float ty, float tz, float rfx, float rfy, float rfz, float rux, float ruy, float ruz) :
		tX(tx), tY(ty), tZ(tz), rfX(rfx), rfY(rfy), rfZ(rfz), ruX(rux), ruY(ruy), ruZ(ruz) {}
	float tX, tY, tZ;
	float rfX, rfY, rfZ;
	float ruX, ruY, ruZ;
};



class Estimator
{
public:

	cv::VideoCapture _capture;
	const string caffe_config_file = "./deploy.prototxt";
	const string caffe_weight_file = "./res10_300x300_ssd_iter_140000_fp16.caffemodel";
	const string landmarks_model = "./shape_predictor_68_face_landmarks.dat";

	// Face box data
	int face_width;
	int center_x;
	int center_y;

	// Tick counter
	int run_count;

	// Capture Dimensions
	int frame_width;
	int frame_height;

	int scale_ratio;

	// Networks
	cv::dnn::Net box_detector;
	dlib::shape_predictor landmark_detector;

	// Storage for reusable variables
	cv::Point2f prev_nose;
	vector< vector<cv::Point2f> > landmarks;
	dlib::rectangle face_rect;
	cv::Mat frame;
	std::vector<cv::Point3d> model_points;

public:

	Estimator();

	int init(int& outCameraWidth, int& outCameraHeight, int detectRatio, int camId);

	void close();

	void detect(TransformData& outFaces);

	void getRawImageBytes(unsigned char* data, int width, int height);

};


