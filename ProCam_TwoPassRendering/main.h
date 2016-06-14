#ifndef MAIN_H
#define MAIN_H

#include <gl/glew.h>
#include <gl/freeglut.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <direct.h>
#include <Windows.h>

// OpenCV��pragma��
#ifdef _DEBUG
#define CV_EXT_STR "d.lib"
#else _REREASE
#define CV_EXT_STR ".lib"
#endif
#define CV_VERSION_STR CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)

#pragma comment(lib,"opencv_core" CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib,"opencv_imgproc" CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib,"opencv_highgui" CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib,"opencv_video" CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib,"opencv_calib3d" CV_VERSION_STR CV_EXT_STR)


// glew��pragma��
#pragma comment(lib,"glew32.lib")

#pragma comment(lib,"FlyCapture2.lib")
#pragma comment(lib,"FlyCapture2GUI.lib")

//�v���W�F�N�^��ʔԍ�
#define PROJECT_MONITOR_NUMBER (1)

// �𑜓x�̐ݒ�
#define PROJECTOR_WIDTH 1280
#define PROJECTOR_HEIGHT 800
#define CAMERA_WIDTH 1920
#define CAMERA_HEIGHT 1200
#define DISPLAY_WIDTH 1680
#define DISPLAY_HEIGHT 0

// �p�X�̐ݒ�
#define PROJECTION_SURFACE_ADDRESS "./Data/cap.jpg"
#define IMAGE_DIRECTORY "./ProjectionImage"

// �L�����u���[�V�����t�@�C���p�X
#define PROCAM_CALIBRATION_RESULT_FILENAME "./Data/calibration.xml"
#define PGRToPGR_CALIBRATION_RESULT_FILENAME "./Data/stereoCameraCalibration_PGR.xml"

#endif