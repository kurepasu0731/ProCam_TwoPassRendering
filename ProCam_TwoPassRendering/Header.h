#ifndef HEADER_H
#define HEADER_H

#pragma once

#include <Windows.h>
#include <opencv2\opencv.hpp>
#include <filesystem>	//std::tr2::sys::path etc

namespace Projection{


	typedef struct disp_prop{
		int index;
		int x, y, width, height;
	} Disp_Prop;

	static int dispCount = -1;
	static std::vector<Disp_Prop> Disps_Prop;

	// �f�B�X�v���C�̏�����
	inline BOOL CALLBACK DispEnumProc(HMONITOR hMonitor, HDC hdcMonitor, LPRECT lprcMonitor, LPARAM dwData) {
		Disp_Prop di;
		di.index = dispCount++;
		di.x = lprcMonitor->left;
		di.y = lprcMonitor->top;
		di.width = lprcMonitor->right - di.x;
		di.height = lprcMonitor->bottom - di.y;
		Disps_Prop.push_back(di);

		return TRUE; // TRUE�͒T���p���CFALSE�ŏI��
	}

	// �f�B�X�v���C���o
	inline void SearchDisplay(void) {
		// ��x�������s����
		if (dispCount == -1) {
			dispCount = 0;
			Disps_Prop = std::vector<Disp_Prop>();
			EnumDisplayMonitors(NULL, NULL, DispEnumProc, 0);
			Sleep(200);
		}
	}

	// �t���X�N���[���œ��e����
	inline void MySetFullScrean(const int num, const char *windowname){

		HWND windowHandle = ::FindWindowA(NULL, windowname);

		SearchDisplay();

		if (NULL != windowHandle) {

			//-�E�B���h�E�X�^�C���ύX�i���j���[�o�[�Ȃ��A�őO�ʁj-
			SetWindowLongPtr(windowHandle, GWL_STYLE, WS_POPUP);
			SetWindowLongPtr(windowHandle, GWL_EXSTYLE, WS_EX_TOPMOST);

			//-�ő剻����-
			ShowWindow(windowHandle, SW_MAXIMIZE);
			cv::setWindowProperty(windowname, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

			//-�f�B�X�v���C���w��-
			Disp_Prop di = Disps_Prop[num];

			//-�N���C�A���g�̈���f�B�X�v���[�ɍ��킹��-
			SetWindowPos(windowHandle, NULL, di.x, di.y, di.width, di.height, SWP_FRAMECHANGED | SWP_NOZORDER);
		}
	}

	// �����ɊY������f�B�X�v���C����Ԃ�
	inline Disp_Prop getDispInfo(int dispNum)
	{
		SearchDisplay();
		// ����������l�̎��͊Y������C���f�b�N�X�ԍ��̃f�B�X�v���C����Ԃ�
		if (dispNum < dispCount) {
			return Disps_Prop[dispNum];
		}
		// ���������������ꍇ�͓K���ȃf�B�X�v���C����Ԃ�
		else{
			Disp_Prop di;
			di.height = 0; di.width = 0;
			return di;
		}
	}

	// ���o�����f�B�X�v���C����Ԃ��֐�
	inline int getDispNum()
	{
		SearchDisplay();
		return Disps_Prop.size();
	}

	// FPS�v�Z
	inline double calcFPS(){
		static int stepCount = 0;
		static int hoge = 0;
		static double fps = 0;
		stepCount++;
		if (GetTickCount64() - hoge >= 1000)
		{
			int elapsed = GetTickCount64() - hoge;
			fps = stepCount / (elapsed / 1000.0);
			// ������2�ʂŊۂ߂�
			fps = (int)(fps * 10) / 10.0;

			std::cout << "LC:" << fps << "[FPS]" << std::endl;
			hoge = GetTickCount64();
			stepCount = 0;
		}
		return fps;
	}
}

#endif