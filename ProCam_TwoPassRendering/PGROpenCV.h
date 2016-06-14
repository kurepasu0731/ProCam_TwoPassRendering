#ifndef PGROPENCV_H
#define PGROPENCV_H

#pragma once

#include <FlyCapture2.h>
#include <opencv2\opencv.hpp>
#include <cstdio>
#include <iostream>

class TPGROpenCV
{
private:
	FlyCapture2::Error			fc2Error;
	FlyCapture2::BusManager		fc2BusMgr;
	FlyCapture2::PGRGuid		fc2Guid;
	FlyCapture2::CameraInfo		fc2CamInfo;
	FlyCapture2::Camera			fc2Cam;
	FlyCapture2::Property		fc2Prop;
	FlyCapture2::PixelFormat	fc2PixelFormat;
	FlyCapture2::Image			fc2Image;

	FlyCapture2::ColorProcessingAlgorithm fc2CPA;
	unsigned int				numCameras;
	unsigned int				useCamIndex;

	float						Shutter_high;
	float						Shutter_middle1;
	float						Shutter_middle2;
	float						Shutter_middle3;
	float						Shutter_low;
	float						Shutter_LC; //�␳���̃V���b�^�[�X�s�[�h
	float						Shutter_PhaseShift; //�␳���̃V���b�^�[�X�s�[�h
	float						Gain;
	float						Gamma;
	float						Brightness;
	float						delay;
	float						Framerate;
	unsigned int				Wb_Red;
	unsigned int				Wb_Blue;
	cv::Mat						fc2Mat;

	void loadParameters();

public:
	TPGROpenCV(int _useCameraIndex = 0);
	~TPGROpenCV();
	int init(FlyCapture2::PixelFormat _format = FlyCapture2::PIXEL_FORMAT_BGR, int ColorProcessingAlgorithm = FlyCapture2::ColorProcessingAlgorithm::HQ_LINEAR);
	void PrintBuildInfo();
	void PrintError(FlyCapture2::Error error);
	void PrintCameraInfo(FlyCapture2::CameraInfo* pCamInfo);
	void InitCameraParameter();
	int PixelFormatInOpenCV();
	int start();
	int queryFrame();
	int stop();
	int release();
	std::string windowNameCamera;

	float getShutter_h() { return Shutter_high; };
	float getShutter_m1(){ return Shutter_middle1; };
	float getShutter_m2(){ return Shutter_middle2; };
	float getShutter_m3(){ return Shutter_middle3; };
	float getShutter_l() { return Shutter_low; };
	float getShutter_LC(){ return Shutter_LC; };
	float getShutter_PS(){ return Shutter_PhaseShift; };
	float getDelay()     { return delay; };

	void setShutterSpeed(float shutterSpeed);
	void setGain(float gain);
	void setWhiteBalance(int r, int b);
	void setPixelFormat(FlyCapture2::PixelFormat format);
	void setGamma(float gamma);
	void setBrightness(float brightness);
	void setColorProcessingAlgorithm(FlyCapture2::ColorProcessingAlgorithm algorithm);
	void setFrameRate(float framerate);
	float getShutterSpeed();
	float getGain();
	void getWhiteBalance(int &r, int &b);
	void showCapImg(cv::Mat cap = cv::Mat());
	void CameraCapture(cv::Mat &image);

	cv::Mat getVideo(){ return fc2Mat; };
};

#endif