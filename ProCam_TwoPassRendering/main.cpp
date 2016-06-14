#include "main.h"
#include "mygl.h"

myGL gl;

// �v���g�^�C�v�錾
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

// �ŏ������Ă΂��֐��i�T�u�X���b�h��run�̒��ŌĂԁj
void initCallFunc()
{
	// �e�R�[���o�b�N�֐��̐ݒ�
	setCallBackFunction();

	gl.initialize();

	// ���[�v�J�n
	glutMainLoop();
}

// �R�[���o�b�N�֐��̐ݒ�
void setCallBackFunction()
{
	Projection::Disp_Prop dispProp;

	// ���j�^�̏����擾
	if (Projection::getDispNum() < 0){
		std::cout << "no display" << std::endl;
		exit(0);
	}
	else{
		int num = Projection::getDispNum();
		for (int i = 0; i < num; i++) {
			dispProp = Projection::getDispInfo(i);
			std::cout << "display" << i << " (�𑜓x): " << dispProp.width << " * " << dispProp.height << std::endl;
		}
	}

	// Window 0
	gl.createWindow(gl.PRJ);
	dispProp = Projection::getDispInfo(PROJECT_MONITOR_NUMBER);
	//�v���W�F�N�^�Ƀt���X�N���[���ŕ\��
	HDC glDc = wglGetCurrentDC();		//GL�̃f�o�C�X�R���e�L�X�g�n���h���擾
	HWND hWnd = WindowFromDC(glDc);		//�E�B���h�E�n���h���擾
	SetWindowLong(hWnd, GWL_STYLE, WS_POPUP);	//���j���[�o�[�̍폜
	SetWindowPos(hWnd, HWND_TOP, dispProp.x, dispProp.y, dispProp.width, dispProp.height, SWP_SHOWWINDOW); //�E�B���h�E�̑����ƈʒu�ύX
	// window1�̏����X�V
	gl.window[gl.PRJ].positionX = dispProp.x;
	gl.window[gl.PRJ].positionY = dispProp.y; //�E�B���h�E�̈ʒu�̎w��
	gl.window[gl.PRJ].width = dispProp.width;
	gl.window[gl.PRJ].height = dispProp.height; //�E�B���h�E�T�C�Y�̎w��
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


// �N���X���̊֐��͒��ڃR�[���o�b�N�Ɏg���Ȃ��̂ŕ��ʂ̊֐������
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