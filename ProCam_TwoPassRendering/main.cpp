#include "main.h"
#include "mygl.h"

myGL gl;

// プロトタイプ宣言
void initCallFunc();
void setCallBackFunction();
void display_prj();
void display_cam();
void display_user();
void reshape_prj(int, int);
void reshape_cam(int, int);
void reshape_user(int, int);
void idle();
void mouseClick(int button, int state, int x, int y);
void mouseMotion(int x, int y);
void mouseWheel(int wheel_number, int direction, int x, int y);
void keyboard(unsigned char key, int x, int y);
void specialKey(int key, int x, int y);
void specialKey_user(int key, int x, int y);
void timer(int value);
void close();


int main(int argc, char *argv[])
{
	glutInit(&argc, argv);
	initCallFunc();
	return 0;
}

// 最初だけ呼ばれる関数（サブスレッドのrunの中で呼ぶ）
void initCallFunc()
{
	// 各コールバック関数の設定
	setCallBackFunction();

	gl.initialize();

	// ループ開始
	glutMainLoop();
}

// コールバック関数の設定
void setCallBackFunction()
{
	Projection::Disp_Prop dispProp;

	// モニタの情報を取得
	if (Projection::getDispNum() < 0){
		std::cout << "no display" << std::endl;
		exit(0);
	}
	else{
		int num = Projection::getDispNum();
		for (int i = 0; i < num; i++) {
			dispProp = Projection::getDispInfo(i);
			std::cout << "display" << i << " (解像度): " << dispProp.width << " * " << dispProp.height << std::endl;
		}
	}

	// Window 0
	gl.createWindow(gl.PRJ);
	dispProp = Projection::getDispInfo(PROJECT_MONITOR_NUMBER);
	//プロジェクタにフルスクリーンで表示
	HDC glDc = wglGetCurrentDC();		//GLのデバイスコンテキストハンドル取得
	HWND hWnd = WindowFromDC(glDc);		//ウィンドウハンドル取得
	SetWindowLong(hWnd, GWL_STYLE, WS_POPUP);	//メニューバーの削除
	SetWindowPos(hWnd, HWND_TOP, dispProp.x, dispProp.y, dispProp.width, dispProp.height, SWP_SHOWWINDOW); //ウィンドウの属性と位置変更
	// window1の情報を更新
	gl.window[gl.PRJ].positionX = dispProp.x;
	gl.window[gl.PRJ].positionY = dispProp.y; //ウィンドウの位置の指定
	gl.window[gl.PRJ].width = dispProp.width;
	gl.window[gl.PRJ].height = dispProp.height; //ウィンドウサイズの指定
	glutDisplayFunc(display_prj);
	glutReshapeFunc(reshape_prj);
	glutIdleFunc(idle);
	glutKeyboardFunc(keyboard);
	glutSpecialFunc(specialKey);
	gl.initGLsetting(gl.PRJ);

	// Window 1
	gl.createWindow(gl.CAM);
	glutDisplayFunc(display_cam);
	glutReshapeFunc(reshape_cam);
	glutIdleFunc(idle);
	glutMouseFunc(mouseClick);
	glutMotionFunc(mouseMotion);
	glutMouseWheelFunc(mouseWheel);
	glutTimerFunc(unsigned int(0.01 * 1000), timer, 0);
	glutCloseFunc(close);
	gl.initGLsetting(gl.CAM);

	// Window 2
	gl.createWindow(2);
	glutDisplayFunc(display_user);
	glutReshapeFunc(reshape_user);
	glutIdleFunc(idle);
	gl.initGLsetting(gl.USER);
	glutSpecialFunc(specialKey_user);

	SetActiveWindow(hWnd);

}


// クラス内の関数は直接コールバックに使えないので普通の関数を介する
void display_prj()
{
	gl.display_projector_view();
}
void display_cam()
{
	gl.display_camera_view();
}
void display_user()
{
	gl.display_user_view();
}
void reshape_prj(int w, int h)
{
	gl.reshape_projector_view(w, h);
}
void reshape_cam(int w, int h)
{
	gl.reshape_camera_view(w, h);
}
void reshape_user(int w, int h)
{
	gl.reshape_user_view(w, h);
}

void idle()
{
	gl.idle();
}

void mouseClick(int button, int state, int x, int y)
{
	gl.mouseClick(button, state, x, y);
}

void mouseMotion(int x, int y)
{
	gl.mouseMotion(x, y);
}

void mouseWheel(int wheel_number, int direction, int x, int y)
{
	gl.mouseWheel(wheel_number, direction, x, y);
}

void keyboard(unsigned char key, int x, int y)
{
	gl.keyboard(key, x, y);
}

void specialKey(int key, int x, int y)
{
	gl.special_key(key, x, y);
}
void specialKey_user(int key, int x, int y)
{
	gl.special_key_user(key, x, y);
}
void timer(int value){
	gl.timer(value);
	glutTimerFunc(unsigned int(0.01 * 1000), timer, 0);
}

void close(){
	gl.close();
}