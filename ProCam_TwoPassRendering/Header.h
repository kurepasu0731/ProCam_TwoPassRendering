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

	// ディスプレイの情報入手
	inline BOOL CALLBACK DispEnumProc(HMONITOR hMonitor, HDC hdcMonitor, LPRECT lprcMonitor, LPARAM dwData) {
		Disp_Prop di;
		di.index = dispCount++;
		di.x = lprcMonitor->left;
		di.y = lprcMonitor->top;
		di.width = lprcMonitor->right - di.x;
		di.height = lprcMonitor->bottom - di.y;
		Disps_Prop.push_back(di);

		return TRUE; // TRUEは探索継続，FALSEで終了
	}

	// ディスプレイ検出
	inline void SearchDisplay(void) {
		// 一度だけ実行する
		if (dispCount == -1) {
			dispCount = 0;
			Disps_Prop = std::vector<Disp_Prop>();
			EnumDisplayMonitors(NULL, NULL, DispEnumProc, 0);
			Sleep(200);
		}
	}

	// フルスクリーンで投影する
	inline void MySetFullScrean(const int num, const char *windowname){

		HWND windowHandle = ::FindWindowA(NULL, windowname);

		SearchDisplay();

		if (NULL != windowHandle) {

			//-ウィンドウスタイル変更（メニューバーなし、最前面）-
			SetWindowLongPtr(windowHandle, GWL_STYLE, WS_POPUP);
			SetWindowLongPtr(windowHandle, GWL_EXSTYLE, WS_EX_TOPMOST);

			//-最大化する-
			ShowWindow(windowHandle, SW_MAXIMIZE);
			cv::setWindowProperty(windowname, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

			//-ディスプレイを指定-
			Disp_Prop di = Disps_Prop[num];

			//-クライアント領域をディスプレーに合わせる-
			SetWindowPos(windowHandle, NULL, di.x, di.y, di.width, di.height, SWP_FRAMECHANGED | SWP_NOZORDER);
		}
	}

	// 引数に該当するディスプレイ情報を返す
	inline Disp_Prop getDispInfo(int dispNum)
	{
		SearchDisplay();
		// 引数が正常値の時は該当するインデックス番号のディスプレイ情報を返す
		if (dispNum < dispCount) {
			return Disps_Prop[dispNum];
		}
		// 引数がおかしい場合は適当なディスプレイ情報を返す
		else{
			Disp_Prop di;
			di.height = 0; di.width = 0;
			return di;
		}
	}

	// 検出したディスプレイ数を返す関数
	inline int getDispNum()
	{
		SearchDisplay();
		return Disps_Prop.size();
	}

	// FPS計算
	inline double calcFPS(){
		static int stepCount = 0;
		static int hoge = 0;
		static double fps = 0;
		stepCount++;
		if (GetTickCount64() - hoge >= 1000)
		{
			int elapsed = GetTickCount64() - hoge;
			fps = stepCount / (elapsed / 1000.0);
			// 小数第2位で丸める
			fps = (int)(fps * 10) / 10.0;

			std::cout << "LC:" << fps << "[FPS]" << std::endl;
			hoge = GetTickCount64();
			stepCount = 0;
		}
		return fps;
	}
}

#endif