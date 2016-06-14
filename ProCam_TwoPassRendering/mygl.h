#ifndef MYGL_H
#define MYGL_H

#include "main.h"
#include "Graycode.h"
#include "Calibration.h"
#include "myTexture.h"
#include "PGROpenCV.h"

class myGL
{
public:

	myGL();
	~myGL();

	GRAYCODE gc;
	Calibration calib;
	TPGROpenCV *pgrOpenCV;
	MyTexture myTex;

	typedef struct _Windows {
		int id;
		int positionX;	// �E�B���h�E�ʒu��X���W
		int positionY;	// �E�B���h�E�ʒu��Y���W
		int width;		// �E�B���h�E�̕�
		int height;		// �E�B���h�E�̍���
		char* title;		// �E�B���h�E�̃^�C�g��
		int flag;			//�E�B���h�E��on off
	} Windows;

	typedef struct _GLMatrix {
		double projectionMatrix[16];
		float textureMatrix[4][4];
		int width;
		int height;
	} GLMatrix;

	// OpenGL�̕ϊ��s����܂Ƃ߂邽�߂̌^
	typedef struct _DeviceMatrix {
		GLMatrix cam;
		GLMatrix proj;
		GLMatrix user;
	} DeviceMatrix;

	typedef enum opengl_window{
		PRJ, CAM, USER
	};

	// OpenGL�̕ϊ��s����܂Ƃ߂Ă���ϐ�
	DeviceMatrix *devmatrix;

	static const int WindowNum = 3;
	Windows window[WindowNum];

	//�J�����ƃv���W�F�N�^�̏��
	const static int CamWidth = CAMERA_WIDTH;
	const static int CamHeight = CAMERA_HEIGHT;
	const static int ProjWidth = PROJECTOR_WIDTH;
	const static int ProjHeight = PROJECTOR_HEIGHT;
	
	// �t���O
	bool exeGeomtric;

	// �R�[���o�b�N�֐��ŌĂ΂���֐�
	void display_projector_view();
	void display_camera_view();
	void display_user_view();
	void reshape_projector_view(int w, int h);
	void reshape_camera_view(int w, int h);
	void reshape_user_view(int w, int h);
	void idle();
	void mouseClick(int button, int state, int x, int y);
	void mouseMotion(int x, int y);
	void mouseWheel(int wheel_number, int direction, int x, int y);
	void keyboard(unsigned char key, int x, int y);
	void special_key(int key, int x, int y);		// window0 ����p
	void special_key_user(int key, int x, int y);	//window 3 ����p
	void close();
	void timer(int value);
	void polarview();

	/**************/
	/* �����o�֐� */
	/**************/

	// �������Ŏg���֐�
	void initWindow();					// window�̈ʒu��T�C�Y���w�肷��֐�
	void createWindow(int window_num);	// window�̏����ݒ���s���֐�
	void initGLsetting(int window_num);	// gl�̏����ݒ�
	void initialize();					// �v���O�����̏����ݒ���s���֐�
	void calcGLMatrix(GLMatrix &myglmatrix, cv::Mat &perspectiveMatrix, int w, int h);	// OpneGL�̕ϊ��s����v�Z����֐�

	// ���̑�
	void setRenderTargetWindow(int window_id);
	void setIdentityMatrix(int viewport_w, int viewpoert_h);					
	void pointCloudRender();													// 3�����_�Q�`��֐�
	void setTexture(cv::Mat &src, GLuint &texture);								// Mat�˃e�N�X�`���ɂ���֐�
	void textureRenderer(GLuint &texture);										// �e�N�X�`���������_�����O����֐�
	std::vector<std::string> getAllFileName(std::string serchDir);				// ���e�p�摜�t�@�C���̓ǂݍ��݂Ɏg�p����֐�
	int loadInputImages(std::string searchDir, std::vector<cv::Mat> &images);	// ���e�p�摜���ꊇ�ǂݍ��݂���֐�
	void saveImage(const unsigned int imageWidth, const unsigned int imageHeight);

	//�@�v���p
	void getPixelCorrespondance(bool projection_image);		//�R�[�h���e����3�����_�Q�擾�܂ōs���֐�
	void getWorldPoint();		// 3�������W���擾����֐�
	void codeProjection();		// �O���C�R�[�h���e���s���֐�
	void smoothing();			// ���������s���֐�
	void convertImageCoordinateUsingTwoPassRendering(cv::Mat &src, cv::Mat &dst, GLMatrix &projector, GLMatrix &camera); // �I�t�X�N���[�������_�����O���s���֐�

private:
	// �x����
	float delay;

	// �Ή��_�̎擾
	std::vector<cv::Point2f> imagePoint;	// �J�������W
	std::vector<cv::Point2f> projPoint;		// �v���W�F�N�^���W
	std::vector<cv::Point3i> pointColor;	// �J���[���
	std::vector<cv::Point3f> reconstructPoint;	// 3�������W

	// �����_�����O�p�e�N�X�`��
	GLuint projectionTexture;
	GLuint cam_texture;
	GLuint user_texture;

	// �摜�f�[�^
	cv::Mat targetImage;		//�ڕW�摜
	cv::Mat capImage;			//�B�e�摜
	cv::Mat distortProjImg;		//�c�܂����v���W�F�N�^�摜
	cv::Mat undistCamImage;		//two pass�p�␳�摜
	cv::Mat undistProjImage;	//two pass�p�␳�摜

	// �c�݌W��
	cv::Mat proj_dist_inv, projDistMap1, projDistMap2, camUndistMap1, camUndistMap2;

	// �摜�t�@�C���ꊇ�擾�p
	std::vector<cv::Mat> originalImages;	
	std::vector < std::string > fileName;
	int fileNum;
	int targetIndex;	// ���ݓ��e���̉摜�ԍ�

	// ���[�U�ʒu�ւ̕��i
	cv::Point3f userTransVec;
	cv::Mat userPos;
	cv::Mat calcUserPerspectiveMatrix(cv::Point3f user_T);	// ���[�U���_�̓������e�s����v�Z����֐�
};



#endif
