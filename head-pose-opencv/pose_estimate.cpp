#pragma once

#include <iostream>
#include <stdio.h>
#include <cstdio>

#include "pose_estimate.h"
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


Estimator::Estimator()
{

	// Load networks
	box_detector = cv::dnn::readNetFromCaffe(caffe_config_file, caffe_weight_file);
	landmark_detector = dlib::shape_predictor();
	dlib::deserialize(landmarks_model) >> landmark_detector;

	// Set starting face box value
	face_rect = dlib::rectangle(dlib::point(0, 0), dlib::point(1, 1));

	// Average human face 3d points (Nose IS longer than average)
	model_points.push_back(cv::Point3d(0.0f, 0.0f, 60.f));               // Nose tip
	model_points.push_back(cv::Point3d(0.0f, -330.0f, -65.0f));          // Chin
	model_points.push_back(cv::Point3d(-225.0f, 170.0f, -135.0f));       // Left eye left corner
	model_points.push_back(cv::Point3d(225.0f, 170.0f, -135.0f));        // Right eye right corner
	model_points.push_back(cv::Point3d(-150.0f, -150.0f, -125.0f));      // Left Mouth corner
	model_points.push_back(cv::Point3d(150.0f, -150.0f, -125.0f));       // Right mouth corner

	// Set resolution
	frame_width = 1920;
	frame_height = 1080;
	scale_ratio = 1;
	run_count = 0;
}

int Estimator::init(int& outCameraWidth, int& outCameraHeight, int detectRatio)
{
	scale_ratio = detectRatio;
	// Open the stream.
	_capture.open(1);
	if (!_capture.isOpened())
		return -2;

	_capture.set(cv::CAP_PROP_FRAME_WIDTH, frame_width);
	_capture.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);
	outCameraWidth = _capture.get(cv::CAP_PROP_FRAME_WIDTH);
	outCameraHeight = _capture.get(cv::CAP_PROP_FRAME_HEIGHT);
	return 0;
}

void Estimator::close()
{
	_capture.release();
}

void Estimator::detect(TransformData& outFaces)
{
	_capture >> frame;
	if (frame.empty())
		return;

	// Convert frame to blob, and drop into Face Box Detector Netowrk
	cv::Mat blob, out;
	blob = cv::dnn::blobFromImage(frame, 1.0, cv::Size(300, 300), (104, 117, 123), false, false);

	box_detector.setInput(blob);
	cv::Mat detection = box_detector.forward();
	cv::Mat detectionMat(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());

	// Check results and only take the most confident prediction
	float largest_conf = 0;
	for (int i = 0; i < detectionMat.rows; i++)
	{
		float confidence = detectionMat.at<float>(i, 2);

		if (confidence > .5)
		{
			// Get dimensions
			int x1 = static_cast<int>(detectionMat.at<float>(i, 3) * (frame_width / scale_ratio));
			int y1 = static_cast<int>(detectionMat.at<float>(i, 4) * (frame_height / scale_ratio));
			int x2 = static_cast<int>(detectionMat.at<float>(i, 5) * (frame_width / scale_ratio));
			int y2 = static_cast<int>(detectionMat.at<float>(i, 6) * (frame_height / scale_ratio));

			// Generate square dimensions
			face_width = max(x2 - x1, y2 - y1) / 2.8;
			center_x = ((x2 + x1) / 2);
			center_y = ((y2 + y1) / 2);

			if (run_count > 0)
			{
				// Average the center of the box with the Nose (Works better for landmark detection)
				center_x = int((center_x + prev_nose.x) / 2);
				center_y = int((center_y + prev_nose.y) / 2);
			}

			// Apply square dimensions
			dlib::point point_a(center_x - face_width, center_y - face_width);
			dlib::point point_b(center_x + face_width, center_y + face_width);
			dlib::rectangle new_face(point_a, point_b);

			if (confidence > largest_conf)
			{
				largest_conf = confidence;
				face_rect = new_face;
			}

		}
	}

	// Run landmark detection
	cv::Mat half_frame(frame_height / scale_ratio, frame_width / scale_ratio, frame.type());
	cv::resize(frame, half_frame, half_frame.size(), cv::INTER_CUBIC);
	dlib::cv_image<dlib::bgr_pixel> dlib_image(half_frame);
	dlib::full_object_detection face_landmark;
	face_landmark = landmark_detector(dlib_image, face_rect);

	// Store nose point
	prev_nose = cv::Point2f(face_landmark.part(34).x(), face_landmark.part(34).y());

	// Prepair face points for perspective solve
	vector<cv::Point2d> image_points;
	image_points.push_back(cv::Point2d(face_landmark.part(30).x() * scale_ratio, face_landmark.part(30).y() * scale_ratio));    // Nose tip
	image_points.push_back(cv::Point2d(face_landmark.part(8).x() * scale_ratio, face_landmark.part(8).y() * scale_ratio));      // Chin
	image_points.push_back(cv::Point2d(face_landmark.part(36).x() * scale_ratio, face_landmark.part(36).y() * scale_ratio));    // Left eye left corner
	image_points.push_back(cv::Point2d(face_landmark.part(45).x() * scale_ratio, face_landmark.part(45).y() * scale_ratio));    // Right eye right corner
	image_points.push_back(cv::Point2d(face_landmark.part(48).x() * scale_ratio, face_landmark.part(48).y() * scale_ratio));    // Left Mouth corner
	image_points.push_back(cv::Point2d(face_landmark.part(54).x() * scale_ratio, face_landmark.part(54).y() * scale_ratio));    // Right mouth corner

	// Generate fake camera Matrix
	double focal_length = frame.cols;
	cv::Point2d center = cv::Point2d(frame.cols / 2, frame.rows / 2);
	cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << focal_length, 0, center.x, 0, focal_length, center.y, 0, 0, 1);
	cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, cv::DataType<double>::type);

	// Output rotation and translation
	cv::Mat rotation_vector;
	cv::Mat translation_vector;
	cv::Mat rot_mat;

	// Solve for pose
	cv::solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector);

	// Convert rotation to Matrix
	cv::Rodrigues(rotation_vector, rot_mat);

	// Export transform
	outFaces = TransformData(translation_vector.at<double>(0), translation_vector.at<double>(1), translation_vector.at<double>(2),
		rot_mat.at<double>(2, 0), rot_mat.at<double>(2, 1), rot_mat.at<double>(2, 2),
		rot_mat.at<double>(1, 0), rot_mat.at<double>(1, 1), rot_mat.at<double>(1, 2));

	run_count += 1;
}


void Estimator::getRawImageBytes(unsigned char* data, int width, int height)
{
	if (frame.empty())
		return;

	cv::Mat tex_mat(height, width, frame.type());
	cv::resize(frame, tex_mat, tex_mat.size(), cv::INTER_CUBIC);

	//Convert from RGB to ARGB 
	cv::Mat argb_img;
	cv::cvtColor(tex_mat, argb_img, cv::COLOR_RGB2BGRA);
	vector<cv::Mat> bgra;
	cv::split(argb_img, bgra);
	cv::swap(bgra[0], bgra[3]);
	cv::swap(bgra[1], bgra[2]);
	// Copy data back to pointer
	memcpy(data, argb_img.data, argb_img.total() * argb_img.elemSize());
}




