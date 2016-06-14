#include "mygl.h"

myGL::myGL()
{

	// �L�����u���[�V�����f�[�^�̓ǂݍ���
	calib.loadCalibParam(PROCAM_CALIBRATION_RESULT_FILENAME);

	// �p�����[�^������
	devmatrix = new DeviceMatrix();
	pgrOpenCV = new TPGROpenCV(0);
	imagePoint.clear();
	projPoint.clear();
	pointColor.clear();
	reconstructPoint.clear();

	// �p�����[�^�̓ǂݍ���
	char buf[128];
	GetPrivateProfileStringA("delay", "delay", NULL, buf, sizeof(buf), "./parameter.ini");
	delay = double(atoi(buf));

	// �e��E�C���h�E�̐ݒ�
	initWindow();

	// �J�������̕ϊ��s��̌v�Z
	cv::Mat perspectiveMatrix = calib.getCamPerspectiveMat();
	calcGLMatrix(devmatrix->cam, perspectiveMatrix, CamWidth, CamHeight);

	// �v���W�F�N�^���̕ϊ��s��̌v�Z
	perspectiveMatrix = calib.getProjPerspectiveMat();
	calcGLMatrix(devmatrix->proj, perspectiveMatrix, ProjWidth, ProjHeight);


	// �t���O�ݒ�
	exeGeomtric = false;

	// �����_�p�̃L�����u���[�V�����f�[�^�̓ǂݍ���
	//calib.loadPGR2PGRCalibParam(PGRToPGR_CALIBRATION_RESULT_FILENAME);

}

myGL::~myGL()
{
	pgrOpenCV->stop();
	pgrOpenCV->release();
	delete pgrOpenCV;
	delete[] devmatrix;
}

/*
** window���̐ݒ���s��
*/
void myGL::initWindow()
{
	// Window 0 �̐ݒ�
	window[PRJ].flag = 1;
	window[PRJ].positionX = 1681;
	window[PRJ].positionY = 0;
	window[PRJ].width = ProjWidth;
	window[PRJ].height = window[PRJ].width * ((double)ProjHeight / (double)ProjWidth);
	window[PRJ].title = "Projector View";

	// Window 1 �̐ݒ�
	window[CAM].flag = 0;
	window[CAM].positionX = 0;
	window[CAM].positionY = 600;
	window[CAM].width = 400;
	window[CAM].height = window[CAM].width * ((double)CamHeight / (double)CamWidth);
	window[CAM].title = "Camera View";

	// Window 2 �̐ݒ�
	window[USER].flag = 0;
	window[USER].positionX = 800;
	window[USER].positionY = 600;
	window[USER].width = 400;
	window[USER].height = window[USER].width * ((double)CamHeight / (double)CamWidth);
	window[USER].title = "User View";
}

/*
** window�̏����ݒ���s���֐�
*/
void myGL::createWindow(int window_num)
{
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);	//�\�����[�h
	glutInitWindowPosition(window[window_num].positionX, window[window_num].positionY); //�E�B���h�E�̈ʒu�̎w��
	glutInitWindowSize(window[window_num].width, window[window_num].height); //�E�B���h�E�T�C�Y�̎w��
	window[window_num].id = glutCreateWindow(window[window_num].title); //�E�B���h�E�̖��O
}

/*
** �ŏ��Ɉ�x�����Ă΂��֐�
*/
void myGL::initialize()
{
	// �J�������X�^�[�g������
	if (pgrOpenCV->init(FlyCapture2::PIXEL_FORMAT_BGR, FlyCapture2::HQ_LINEAR) != -1) {	// �J����������ɓ��삵�Ă����ꍇ
		pgrOpenCV->setShutterSpeed(pgrOpenCV->getShutter_h());
		pgrOpenCV->start();	// �J�����X�^�[�g
	}
	else {
		std::cout << "camera connection error" << std::endl;
		exit(-1);
	}

	// �J�����A�v���W�F�N�^�̘c�݃}�b�v�̎Z�o
	proj_dist_inv = calib.proj_dist * -1;	// �v���W�F�N�^���̃����_�����O���ʂ�c�܂���s��
	cv::initUndistortRectifyMap(calib.cam_K, calib.cam_dist, cv::Mat(), calib.cam_K, cv::Size(CamWidth, CamHeight), CV_32FC1, camUndistMap1, camUndistMap2);
	cv::initUndistortRectifyMap(calib.proj_K, proj_dist_inv, cv::Mat(), calib.proj_K, cv::Size(ProjWidth, ProjHeight), CV_32FC1, projDistMap1, projDistMap2);

	// �摜�̓ǂݍ���
	targetIndex = 0;
	if (loadInputImages(IMAGE_DIRECTORY, originalImages) == -1){
		std::cout << "���e�p�̉摜�f�[�^������܂���" << std::endl;
		exit(-1);
	}
	targetImage = originalImages[targetIndex];

	// �����_�Ή��p�̕ϐ�
	userTransVec = cv::Point3f(0, 0, 0);	// ���[�U�̏����ʒu�x�N�g���i�P��[mm]�j
	userPos = cv::Mat(4, 1, CV_64F);		// ���[�U�̈ʒu�x�N�g�����i�[����mat (�������W�\��)

	std::cout << "r�F���Z�b�g" << std::endl;
	std::cout << "q�F�I��" << std::endl;
	std::cout << "0�F�R�[�h���e����" << std::endl;
	std::cout << "1�F�R�[�h���e�Ȃ�" << std::endl;
	std::cout << "�� : �摜�ύX" << std::endl;
	std::cout << "�� : �摜�ύX" << std::endl;
}
/*
** openGL�̕ϊ��s����v�Z����֐�
*/
void myGL::calcGLMatrix(GLMatrix &myglmatrix, cv::Mat &perspectiveMatrix, int w, int h)
{
	// �J�������_�̕ϊ��s��̌v�Z
	double CONVERT_TO_OPENGL_WIN[16] = {
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1
	};

	// �H�N���b�s���O�̈�̐��K��+���W�ϊ�
	CONVERT_TO_OPENGL_WIN[0] = 1.0 / (w / 2.0);
	CONVERT_TO_OPENGL_WIN[5] = -1.0 / (h / 2.0);
	CONVERT_TO_OPENGL_WIN[12] = -1.0;
	CONVERT_TO_OPENGL_WIN[13] = 1.0;

	// �ˉe�ϊ��s��̐ݒ�
	double projection[16];
	for (int i = 0; i < 4; i++){
		projection[i * 4] = perspectiveMatrix.at<double>(0, i);
		projection[i * 4 + 1] = perspectiveMatrix.at<double>(1, i);
		projection[i * 4 + 2] = 0;
		projection[i * 4 + 3] = perspectiveMatrix.at<double>(2, i);
	}

	// OpenGL��GL_PROJECTION ���[�h�̂Ƃ��Ɏg�p����ˉe�ϊ��s����v�Z
	cv::Mat OpenGLWin(4, 4, CV_64FC1, CONVERT_TO_OPENGL_WIN);
	cv::Mat ProjctionMat(4, 4, CV_64FC1, projection);
	ProjctionMat = ProjctionMat * OpenGLWin; // �s���]�u�����OpenGL�̔z��̏��Ԃƈ�v
	for (int i = 0; i < 4; i++){
		for (int j = 0; j < 4; j++){
			myglmatrix.projectionMatrix[i * 4 + j] = ProjctionMat.at<double>(i, j); // ���X�^�[�X�L����
		}
	}

	// �e�N�X�`���}�g���b�N�X���v�Z
	static double modelview[16] = {
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1
	};
	// �e�N�X�`�����W�n�ɂ��킹��s��
	double convert[16] = {
		0.5, 0, 0, 0.5,
		0, -0.5, 0, 0.5,
		0, 0, 1, 0,
		0, 0, 0, 1
	};
	cv::Mat Model(4, 4, CV_64FC1, modelview);
	cv::Mat Texture(4, 4, CV_64FC1, convert);
	cv::Mat PM = Model * ProjctionMat;
	cv::transpose(PM, PM); //�]�u
	PM = Texture * PM;
	int index = 0;
	for (int i = 0; i < 4; i++){
		for (int t = 0; t < 4; t++){
			myglmatrix.textureMatrix[i][t] = cv::saturate_cast<double>(PM.at<double>(i, t));
		}
	}

	// �c���𑜓x��ێ�
	myglmatrix.width = w;
	myglmatrix.height = h;
}

/*
** Window���Ƃ�OpenGL�̐ݒ���s���֐�
*/
void myGL::initGLsetting(int window_num)
{

	/* Window1 (�v���W�F�N�^���_�f��) �̐ݒ� */
	if (window_num == PRJ)
	{
		glClearColor(0.0, 0.0, 0.0, 1.0);
		glEnable(GL_DEPTH_TEST);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		// GLEW �̏�����
		GLenum err = glewInit();
		if (err != GLEW_OK) {
			fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
			exit(1);
		}
	}

	/* Window0 (�J�������_�f��) �̐ݒ� */
	if (window_num == CAM)
	{
		glClearColor(0.0, 0.0, 0.0, 1.0);
		glEnable(GL_DEPTH_TEST);
		// OpenGL�̎ˉe�s���ݒ肷��
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
	}

	/* Window2 (���[�U���_�f��) �̐ݒ� */
	if (window_num == USER)
	{
		glClearColor(0.0, 0.0, 0.0, 1.0);
		glEnable(GL_DEPTH_TEST);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
	}
}

//************************
//		�`��
//************************

/* �v���W�F�N�^���̕`�揈�� */
void myGL::display_projector_view()
{
	if (exeGeomtric == true){

		// twopass rendering �ɂ��􉽕␳�摜���쐬
		cv::remap(targetImage, undistCamImage, camUndistMap1, camUndistMap2, cv::INTER_LINEAR);	// �c�݂̏���
		convertImageCoordinateUsingTwoPassRendering(undistCamImage, undistProjImage, devmatrix->cam, devmatrix->proj); //�v���W�F�N�^�𑜓x��
		cv::remap(undistProjImage, distortProjImg, projDistMap1, projDistMap2, cv::INTER_LINEAR);	// �c�݂�^����
		// �e�N�X�`����
		setTexture(distortProjImg, projectionTexture);

		// �e�ϊ��s���P�ʍs��ɂ���
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glViewport(0, 0, window[PRJ].width, window[PRJ].height);	// �r���[�|�[�g�̓E�B���h�E�̃T�C�Y�ɍ��킹��

		// �␳�摜�̓��e
		textureRenderer(projectionTexture);
		glDeleteTextures(1, &projectionTexture);	// �g���I������e�N�X�`�����폜

		// PGR�B�e�摜�̕\��
		//pgrOpenCV->CameraCapture(capImage);
		//pgrOpenCV->showCapImg(capImage);
	}
	else{
		setIdentityMatrix(window[PRJ].width, window[PRJ].height);
		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glutSwapBuffers();
	}


}

/* �J�������̕`�揈�� */
void myGL::display_camera_view()
{
	if (exeGeomtric == true){

		setTexture(targetImage, cam_texture);	// �e�N�X�`����
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glMultMatrixd(devmatrix->cam.projectionMatrix);

		/* �e�N�X�`���s��̐ݒ� */
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		// �ˉe�}�b�s���O�̈ʒu�����[�U���_�ɐݒ�
		glMatrixMode(GL_TEXTURE);

		// ���������̌v�Z���ɃI�u�W�F�N�g��Ԃ̒��_���W���g���B
		glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
		glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
		glTexGeni(GL_R, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
		glTexGeni(GL_Q, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);

		// ���������ϊ��s����I�u�W�F�N�g�̒��_�Ɋ|����Ή�ʂ𕢂��悤��UV���v�Z�����B
		glTexGenfv(GL_S, GL_OBJECT_PLANE, devmatrix->cam.textureMatrix[0]);
		glTexGenfv(GL_T, GL_OBJECT_PLANE, devmatrix->cam.textureMatrix[1]);
		glTexGenfv(GL_R, GL_OBJECT_PLANE, devmatrix->cam.textureMatrix[2]);
		glTexGenfv(GL_Q, GL_OBJECT_PLANE, devmatrix->cam.textureMatrix[3]);

		// UV�̎���������L��������B
		glEnable(GL_TEXTURE_GEN_S);
		glEnable(GL_TEXTURE_GEN_T);
		glEnable(GL_TEXTURE_GEN_R);  // ���܂�
		glEnable(GL_TEXTURE_GEN_Q);

		/* ��ʃN���A */
		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		//�e�N�X�`���}�b�s���O�J�n
		glBindTexture(GL_TEXTURE_2D, cam_texture);
		glEnable(GL_TEXTURE_2D);
		pointCloudRender();
		//�e�N�X�`���}�b�s���O�I��
		glDisable(GL_TEXTURE_2D);

		// UV�̎���������L��������B
		glDisable(GL_TEXTURE_GEN_S);
		glDisable(GL_TEXTURE_GEN_T);
		glDisable(GL_TEXTURE_GEN_R);
		glDisable(GL_TEXTURE_GEN_Q);

		glutSwapBuffers();
		glDeleteTextures(1, &cam_texture);
	}
	else
	{
		setIdentityMatrix(window[CAM].width, window[CAM].height);
		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glutSwapBuffers();
	}
}

/*
** ���[�U���_�̃����_�����O
*/
void myGL::display_user_view()
{
	if (exeGeomtric == true){

		setTexture(targetImage, user_texture);	// �e�N�X�`����
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glMultMatrixd(devmatrix->user.projectionMatrix);

		/* �e�N�X�`���s��̐ݒ� */
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		// �ˉe�}�b�s���O�̈ʒu�����[�U���_�ɐݒ�
		glMatrixMode(GL_TEXTURE);

		// ���������̌v�Z���ɃI�u�W�F�N�g��Ԃ̒��_���W���g���B
		glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
		glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
		glTexGeni(GL_R, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
		glTexGeni(GL_Q, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);

		// ���������ϊ��s����I�u�W�F�N�g�̒��_�Ɋ|����Ή�ʂ𕢂��悤��UV���v�Z�����B
		glTexGenfv(GL_S, GL_OBJECT_PLANE, devmatrix->cam.textureMatrix[0]);
		glTexGenfv(GL_T, GL_OBJECT_PLANE, devmatrix->cam.textureMatrix[1]);
		glTexGenfv(GL_R, GL_OBJECT_PLANE, devmatrix->cam.textureMatrix[2]);
		glTexGenfv(GL_Q, GL_OBJECT_PLANE, devmatrix->cam.textureMatrix[3]);

		// UV�̎���������L��������B
		glEnable(GL_TEXTURE_GEN_S);
		glEnable(GL_TEXTURE_GEN_T);
		glEnable(GL_TEXTURE_GEN_R);  // ���܂�
		glEnable(GL_TEXTURE_GEN_Q);

		/* ��ʃN���A */
		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		//�e�N�X�`���}�b�s���O�J�n
		glBindTexture(GL_TEXTURE_2D, user_texture);
		glEnable(GL_TEXTURE_2D);
		pointCloudRender();
		//�e�N�X�`���}�b�s���O�I��
		glDisable(GL_TEXTURE_2D);

		// UV�̎���������L��������B
		glDisable(GL_TEXTURE_GEN_S);
		glDisable(GL_TEXTURE_GEN_T);
		glDisable(GL_TEXTURE_GEN_R);
		glDisable(GL_TEXTURE_GEN_Q);

		glutSwapBuffers();
		glDeleteTextures(1, &user_texture);
	}
	else
	{
		setIdentityMatrix(window[CAM].width, window[CAM].height);
		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glutSwapBuffers();
	}
}

/* �A�C�h�����̏��� */
void myGL::idle()
{
	for (int loop = 0; loop < WindowNum; ++loop){
		if (window[loop].flag > 0) {
			glutSetWindow(window[loop].id);
			glutPostRedisplay(); //�ĕ`�� (��display()�֐����Ăяo���֐� )
		}
	}
}

/* �}�E�X�N���b�N���̏��� */
void myGL::mouseClick(int button, int state, int x, int y)
{

}

/* �}�E�X�h���b�O���̏��� */
void myGL::mouseMotion(int x, int y)
{

}

/* �}�E�X�z�C�[�����쎞�̏��� */
void myGL::mouseWheel(int wheel_number, int direction, int x, int y)
{
}

/* ���Ԋu�ŌĂ΂��֐� */
void myGL::timer(int value)
{
}

/* glut�̃��[�v�I�����ɌĂ΂��֐� */
void myGL::close()
{
}

//���_�ύX
void myGL::polarview(){

}

/* ���T�C�Y���̏��� */
void myGL::reshape_camera_view(int w, int h)
{
	window[CAM].width = w;
	window[CAM].height = h;
	glViewport(0, 0, w, h);	//�`��T�C�Y�Œ艻
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMultMatrixd(devmatrix->cam.projectionMatrix);
	glutPostRedisplay();	//glutDisplayFunc()���P����s����
}
void myGL::reshape_projector_view(int w, int h)
{
	window[PRJ].width = w;
	window[PRJ].height = h;
	glViewport(0, 0, w, h);	//�`��T�C�Y�Œ艻
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMultMatrixd(devmatrix->proj.projectionMatrix);
	glutPostRedisplay();	//glutDisplayFunc()���P����s����
}

void myGL::reshape_user_view(int w, int h)
{
	window[USER].width = w;
	window[USER].height = h;
	glViewport(0, 0, w, h);	//�`��T�C�Y�Œ艻
	glutPostRedisplay();	//glutDisplayFunc()���P����s����
}

/* �L�[�{�[�h���쎞�̏��� */
void myGL::keyboard(unsigned char key, int x, int y)
{
	switch (key){
	case 'q':	//�I��
		exit(1);
		break;
	case '0':
		getPixelCorrespondance(1);
		exeGeomtric = true;
		break;
	case '1':
		getPixelCorrespondance(0);
		exeGeomtric = true;
		break;
	}

	std::cout << "*******���͑҂�*********" << std::endl;
}
/*
** ����L�[���͎�t�p
*/
void myGL::special_key(int key, int x, int y)
{
	switch (key){

	case GLUT_KEY_LEFT:
		targetIndex--;
		if (targetIndex < 0) targetIndex = fileNum - 1;
		targetImage = originalImages[targetIndex];
		break;

	case GLUT_KEY_RIGHT:
		targetIndex++;
		if (targetIndex >= fileNum) targetIndex = 0;
		targetImage = originalImages[targetIndex];
		break;

	}
	std::cout << "*******���͑҂�*********" << std::endl;
}
void myGL::special_key_user(int key, int x, int y)
{
	switch (key){

	case GLUT_KEY_LEFT:
		userTransVec.x -= 100;
		break;

	case GLUT_KEY_RIGHT:
		userTransVec.x += 100;
		break;

	case GLUT_KEY_UP:
		userTransVec.y -= 100;
		break;

	case GLUT_KEY_DOWN:
		userTransVec.y += 100;
		break;

	case GLUT_KEY_END:
		break;
	}
	// ���[�U���_�ʒu�ɒu���ꂽ�J�����̓������e�s��̌v�Z(3�������������ザ��Ȃ��ƌv�Z�ł��Ȃ��̂Œ���)
	cv::Mat perspectiveMat = calcUserPerspectiveMatrix(userTransVec);
	// ���[�U���_��GL�ϊ��s��̌v�Z
	calcGLMatrix(devmatrix->user, perspectiveMat, CamWidth, CamHeight);

	std::cout << "*******���͑҂�*********" << std::endl;
}

//*****************************************
//				�p�[�c�Q
//*****************************************

/*
** �����_�����O����E�C���h�E���w�肷��֐�
*/
void myGL::setRenderTargetWindow(int window_id)
{
	// �`�悷��E�C���h�E���Z�b�g
	glutSetWindow(window_id);
}

/*
** OpneGL�̕ϊ��s������Z�b�g����֐�
*/
void myGL::setIdentityMatrix(int viewport_w, int viewpoert_h)
{
	// GL�̕ϊ��s��̏�����
	glViewport(0, 0, viewport_w, viewpoert_h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMatrixMode(GL_TEXTURE);
	glLoadIdentity();
}

/*
** ���e�ʂ�_�Q�Ƃ��ĕ`�悷��
*/
void myGL::pointCloudRender()
{
	glBegin(GL_POINTS);
	glPointSize(1);
	for (int i = 0; i < reconstructPoint.size(); ++i) {
		//�Ή������Ă���Ε`��
		if (projPoint[i].x != -1){
			glColor3f(pointColor[i].x / 255.0f, pointColor[i].y / 255.0f, pointColor[i].z / 255.0f); //RGB
			glVertex3f(reconstructPoint[i].x, reconstructPoint[i].y, reconstructPoint[i].z);
		}
	}
	glEnd();
}


/*
** �e�N�X�`�����쐬����
** �O������Ăяo���p
*/
void myGL::setTexture(cv::Mat &src, GLuint &texture)
{
	myTex.setTexture(src, &texture);
}
/*
** �v���p�̉摜�𓊉e����Ƃ��Ɏg�p���郌���_�����O�֐�
** �֐��Ăяo���O�ɕϊ��s��������l�ɃZ�b�g���Ă�������
*/
void myGL::textureRenderer(GLuint &texture)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	//�o�b�t�@�N���A

	// �A�e�t���ƉB�ʏ��������͍s��Ȃ�
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);
	//glEnable(GL_DEPTH_TEST);

	// �e�N�X�`���}�b�s���O��L���ɂ���
	glBindTexture(GL_TEXTURE_2D, texture);
	glEnable(GL_TEXTURE_2D);

	// �����`��`��
	glColor3d(1.0, 1.0, 1.0);
	glBegin(GL_TRIANGLE_FAN);
	glTexCoord2d(0.0, 1.0);
	glVertex2d(-1.0, -1.0);
	glTexCoord2d(1.0, 1.0);
	glVertex2d(1.0, -1.0);
	glTexCoord2d(1.0, 0.0);
	glVertex2d(1.0, 1.0);
	glTexCoord2d(0.0, 0.0);
	glVertex2d(-1.0, 1.0);
	glEnd();

	// �e�N�X�`���}�b�s���O�𖳌��ɂ���
	glDisable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_DEPTH_TEST);

	glutSwapBuffers();

}


/**
@brief ���ʃf�B���N�g�����̑S�t�@�C���̖��O���擾�i�ꊇ�ϊ��p�j
*/
std::vector<std::string> myGL::getAllFileName(std::string searchDir)
{
	std::vector<std::string> file_list;

	// �J�����g�f�B���N�g���ȉ��̃t�@�C�������擾����
	// �ċA�I�Ƀt�@�C�������擾����ꍇ�́Astd::tr2::sys::recursive_directory_iterator���g��
	for (std::tr2::sys::directory_iterator it(searchDir), end; it != end; ++it) {
		// �摜�t�@�C�������擾
		std::string ext = it->path().extension();
		if (ext == ".jpg" ||
			ext == ".JPG" ||
			ext == ".png" ||
			ext == ".PNG" ||
			ext == ".bmp" ||
			ext == ".BMP"
			){
			file_list.push_back(it->path());
		}
	}

	// �擾�����t�@�C���������ׂĕ\������
	//for (auto &path : file_list) {
	//	std::cout << path << std::endl;
	//}
	return file_list;
}


/*
** ���e����摜��ǂݍ��ފ֐�
*/
int myGL::loadInputImages(std::string searchDir, std::vector<cv::Mat> &images)
{
	// �t�@�C���p�X�̎擾
	fileName = getAllFileName(searchDir);
	// �摜�̓ǂݍ���
	fileNum = fileName.size();
	// �ǂݍ��މ摜��������Ȃ������ꍇ
	if (fileNum == 0){
		std::cout << "no image" << std::endl;
		return -1;
	}
	for (int i = 0; i < fileNum; i++){
		images.emplace_back(cv::imread(fileName[i].c_str()));
	}

	return 0;
}

/*
** �e�N�X�`����ۑ�����֐�
*/
void myGL::saveImage(const unsigned int imageWidth, const unsigned int imageHeight)
{
	const unsigned int channelNum = 3; // RGB�Ȃ�3, RGBA�Ȃ�4
	void* dataBuffer = NULL;
	dataBuffer = (GLubyte*)malloc(imageWidth * imageHeight * channelNum);

	// �ǂݎ��OpneGL�̃o�b�t�@���w�� GL_FRONT:�t�����g�o�b�t�@�@GL_BACK:�o�b�N�o�b�t�@
	glReadBuffer(GL_FRONT);

	// OpenGL�ŉ�ʂɕ`�悳��Ă�����e���o�b�t�@�Ɋi�[
	glReadPixels(
		0,                 //�ǂݎ��̈�̍�������x���W
		0,                 //�ǂݎ��̈�̍�������y���W //0 or getCurrentWidth() - 1
		imageWidth,             //�ǂݎ��̈�̕�
		imageHeight,            //�ǂݎ��̈�̍���
		GL_BGR, //it means GL_BGR,           //�擾�������F���̌`��
		GL_UNSIGNED_BYTE,  //�ǂݎ�����f�[�^��ۑ�����z��̌^
		dataBuffer      //�r�b�g�}�b�v�̃s�N�Z���f�[�^�i���ۂɂ̓o�C�g�z��j�ւ̃|�C���^
		);

	GLubyte* p = static_cast<GLubyte*>(dataBuffer);
	std::string fname = "./SaveImage/outputImage.jpg";
	IplImage* outImage = cvCreateImage(cvSize(imageWidth, imageHeight), IPL_DEPTH_8U, 3);

	for (unsigned int j = 0; j < imageHeight; ++j)
	{
		for (unsigned int i = 0; i < imageWidth; ++i)
		{
			outImage->imageData[(imageHeight - j - 1) * outImage->widthStep + i * 3 + 0] = *p;
			outImage->imageData[(imageHeight - j - 1) * outImage->widthStep + i * 3 + 1] = *(p + 1);
			outImage->imageData[(imageHeight - j - 1) * outImage->widthStep + i * 3 + 2] = *(p + 2);
			p += 3;
		}
	}

	cvSaveImage(fname.c_str(), outImage);
}

/*
@brief 2�p�X�����_�����O���I�t�X�N���[���ōs���֐�
@details
@param src �ˉe�}�b�s���O�p�摜
@param dst �����_�����O����
@param projector 1pass�ڂ̃f�o�C�X
@param camera 2pass�ڂ̃f�o�C�X
*/
void myGL::convertImageCoordinateUsingTwoPassRendering(cv::Mat &src, cv::Mat &dst, GLMatrix &projector, GLMatrix &camera)
{
	// �I�t�X�N���[�������_�����O����ۂ̃����_�����O���ʂ̉𑜓x���w��
	int dstWidth = camera.width;
	int dstHeight = camera.height;

	// GL�̕ϊ��s��̏�����
	setRenderTargetWindow(window[PRJ].id);
	setIdentityMatrix(dstWidth, dstHeight);

	// �t���[���o�b�t�@�I�u�W�F�N�g�p�ϐ�
	GLuint fboID, _cb, _rb;
	myTex.setFramebufferTexture(&fboID, &_cb, &_rb, dstWidth, dstHeight);
	GLuint Texture;
	setTexture(src, Texture);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMultMatrixd(camera.projectionMatrix);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// �t���[���o�b�t�@�I�u�W�F�N�g����������
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fboID);

	//�e�N�X�`���}�b�s���O�J�n
	glBindTexture(GL_TEXTURE_2D, Texture);
	glEnable(GL_TEXTURE_2D);

	// UV�̎���������L��������B
	glEnable(GL_TEXTURE_GEN_S);
	glEnable(GL_TEXTURE_GEN_T);
	glEnable(GL_TEXTURE_GEN_R);  // ���܂�
	glEnable(GL_TEXTURE_GEN_Q);

	// ���������̌v�Z���ɃI�u�W�F�N�g��Ԃ̒��_���W���g���B
	glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
	glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
	glTexGeni(GL_R, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
	glTexGeni(GL_Q, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);

	// ���������ϊ��s����I�u�W�F�N�g�̒��_�Ɋ|����Ή�ʂ𕢂��悤��UV���v�Z�����B
	glTexGenfv(GL_S, GL_OBJECT_PLANE, projector.textureMatrix[0]);
	glTexGenfv(GL_T, GL_OBJECT_PLANE, projector.textureMatrix[1]);
	glTexGenfv(GL_R, GL_OBJECT_PLANE, projector.textureMatrix[2]);
	glTexGenfv(GL_Q, GL_OBJECT_PLANE, projector.textureMatrix[3]);

	/* ���f���r���[�ϊ��s��̐ݒ� */
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	///* ��ʃN���A */
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// 3�����`��𕜌�
	pointCloudRender();
	// �e�N�X�`�����W�̎��������𖳌��ɂ���
	glDisable(GL_TEXTURE_GEN_S);
	glDisable(GL_TEXTURE_GEN_T);
	glDisable(GL_TEXTURE_GEN_R);
	glDisable(GL_TEXTURE_GEN_Q);
	//�e�N�X�`���}�b�s���O�I��
	glDisable(GL_TEXTURE_2D);

	///* �e�N�X�`���}�b�s���O�𖳌��ɂ���*/
	glDisable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, 0);

	// �`����s
	glFlush();

	// �e�N�X�`�����摜��
	dst = cv::Mat(cv::Size(dstWidth, dstHeight), CV_8UC3);	//RGBA�łȂ��Ă��悢
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);// FBO : GL_COLOR_ATTACHMENT0_EXT, GL_FRONT, GL_BACK
	glReadPixels(0, 0, dstWidth, dstHeight, GL_BGR, GL_UNSIGNED_BYTE, (void*)dst.data);
	cv::flip(dst, dst, 0);

	// �t���[���o�b�t�@�I�u�W�F�N�g�̌�������������(�摜�ɓǂݍ��񂾌�ɁC�o�C���h����)
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
	myTex.deleteFramebufferTexture(&fboID, &_cb, &_rb);
	glDeleteTextures(1 * 2, &Texture);

	setIdentityMatrix(window[PRJ].width, window[PRJ].height);
}
