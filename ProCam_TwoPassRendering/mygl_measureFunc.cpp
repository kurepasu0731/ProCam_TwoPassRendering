#include "mygl.h"
#include "Header.h"


/*
** �R�[�h���e
*/
void myGL::getPixelCorrespondance(bool projection_image)
{
	//�@�O���C�R�[�h�̓��e���B�e���Ή��_�擾
	if (projection_image){
		codeProjection();
	}
	gc.make_thresh();
	gc.makeCorrespondence();
	getWorldPoint();

	// ���[�U���_�ʒu�ɒu���ꂽ�J�����̓������e�s��̌v�Z(3�������������ザ��Ȃ��ƌv�Z�ł��Ȃ��̂Œ���)
	cv::Mat perspectiveMat = calcUserPerspectiveMatrix(userTransVec);
	// ���[�U���_��GL�ϊ��s��̌v�Z
	calcGLMatrix(devmatrix->user, perspectiveMat, CamWidth, CamHeight);
}


/*
** �\�����p�^�[���𓊉e����֐�
*/
void myGL::codeProjection()
{
	// GL�̕ϊ��s��̏�����
	setRenderTargetWindow(window[PRJ].id);
	setIdentityMatrix(window[PRJ].width, window[PRJ].height);

	// �萔
	typedef enum flag{
		POSI = true,
		NEGA = false,
		VERTICAL = true,
		HORIZONTAL = false,
	} flag;

	//�����ݒ�
	if (pgrOpenCV->init(FlyCapture2::PIXEL_FORMAT_BGR, FlyCapture2::HQ_LINEAR) == -1){
		exit(0);
	}
	pgrOpenCV->setShutterSpeed(pgrOpenCV->getShutter_PS());

	/* --- Gray Code �̓ǂݍ��� --- */
	// ���e����
	int all_bit = gc.c->g.all_bit;
	int h_bit = gc.c->g.h_bit;

	// ���e�摜(�|�W&�l�K)�i�[�p
	cv::Mat *posi_img = new cv::Mat[all_bit];  // �|�W�p�^�[���p
	cv::Mat *nega_img = new cv::Mat[all_bit];  // �l�K�p�^�[���p

	// �`��e�N�X�`���p
	GLuint *posi_texture = new GLuint[all_bit];  // �|�W�p�^�[���p
	GLuint *nega_texture = new GLuint[all_bit];  // �l�K�p�^�[���p

	// �����t���o�́i�O���C�R�[�h�ǂݍ��ݗp�j
	std::stringstream *Filename_posi = new std::stringstream[all_bit];
	std::stringstream *Filename_nega = new std::stringstream[all_bit];
	// �����t���o�́i�B�e�摜�������ݗp�j
	std::stringstream *Filename_posi_cam = new std::stringstream[all_bit];
	std::stringstream *Filename_nega_cam = new std::stringstream[all_bit];

	std::cout << "���e�p�O���C�R�[�h�摜�ǂݍ��ݒ�" << std::endl;
	for (unsigned int i = 0; i < all_bit; i++) {
		Filename_posi[i] << "./GrayCodeImage/ProjectionGrayCode/posi" << std::setw(2) << std::setfill('0') << i << ".bmp";
		Filename_nega[i] << "./GrayCodeImage/ProjectionGrayCode/nega" << std::setw(2) << std::setfill('0') << i << ".bmp";
		// �ǂݍ���
		posi_img[i] = cv::imread(Filename_posi[i].str(), 1);
		nega_img[i] = cv::imread(Filename_nega[i].str(), 1);
		Filename_posi[i] << std::endl;
		Filename_nega[i] << std::endl;

		// �ǂݍ��ޖ���������Ȃ�������O���C�R�[�h�摜����蒼��
		if (posi_img[i].empty() || nega_img[i].empty()){
			std::cout << "ERROR(1)�F���e�p�̃O���C�R�[�h�摜���s�����Ă��܂��B" << std::endl;
			std::cout << "ERROR(2)�F�O���C�R�[�h�摜���쐬���܂��B" << std::endl;
			gc.makeGraycodeImage();
			codeProjection();
			return;
		}
		setTexture(posi_img[i], posi_texture[i]);
		setTexture(nega_img[i], nega_texture[i]);
	}

	// ���F�摜�̍쐬
	cv::Mat white = cv::Mat(ProjHeight, ProjWidth, CV_8UC3, cv::Scalar(255, 255, 255));
	GLuint w_texture;
	setTexture(white, w_texture);

	std::cout << "���e�J�n" << std::endl;

	// posi�摜�̓��e�B�e
	for (unsigned int i = 0; i < all_bit; i++) {
		cv::Mat cap;
		// �����_�����O
		textureRenderer(posi_texture[i]);
		Sleep(delay * 1.5);
		// �B�e�摜��mat�Ɋi�[
		pgrOpenCV->CameraCapture(cap);	//13ms
		// ����
		if (i < h_bit)
			Filename_posi_cam[i] << "./GrayCodeImage/CaptureImage/CameraImg" << HORIZONTAL << "_" << std::setw(2) << std::setfill('0') << i + 1 << "_" << POSI << ".bmp";
		// �c��
		else
			Filename_posi_cam[i] << "./GrayCodeImage/CaptureImage/CameraImg" << VERTICAL << "_" << std::setw(2) << std::setfill('0') << i - h_bit + 1 << "_" << POSI << ".bmp";

		// �ۑ�
		cv::imwrite(Filename_posi_cam[i].str(), cap);
		Filename_posi_cam[i] << std::endl;
	}

	// nega�摜�̓��e�B�e
	for (unsigned int i = 0; i < all_bit; i++) {
		cv::Mat cap;
		// �����_�����O
		textureRenderer(nega_texture[i]);
		Sleep(delay * 1.5);
		// �B�e�摜��mat�Ɋi�[
		pgrOpenCV->CameraCapture(cap);	//13ms
		if (i < h_bit)
			Filename_nega_cam[i] << "./GrayCodeImage/CaptureImage/CameraImg" << HORIZONTAL << "_" << std::setw(2) << std::setfill('0') << i + 1 << "_" << NEGA << ".bmp";
		// �c��
		else
			Filename_nega_cam[i] << "./GrayCodeImage/CaptureImage/CameraImg" << VERTICAL << "_" << std::setw(2) << std::setfill('0') << i - h_bit + 1 << "_" << NEGA << ".bmp";
		// �ۑ�
		cv::imwrite(Filename_nega_cam[i].str(), cap);
		Filename_nega_cam[i] << std::endl;
	}

	cv::Mat cap;
	// ���F�摜�������_�����O
	textureRenderer(w_texture);
	Sleep(delay * 1.5);
	pgrOpenCV->CameraCapture(cap);	//13ms
	cv::imwrite(PROJECTION_SURFACE_ADDRESS, cap);

	glBindTexture(GL_TEXTURE_2D, 0);

	/***** �I�� *****/
	// �e�N�X�`���̏���
	glDeleteTextures(all_bit * 2, posi_texture);
	glDeleteTextures(all_bit * 2, nega_texture);
	glDeleteTextures(1 * 2, &w_texture);
	// ���
	delete[] posi_img;
	delete[] nega_img;
	delete[] Filename_posi;
	delete[] Filename_nega;
	delete[] Filename_posi_cam;
	delete[] Filename_nega_cam;
	delete[] posi_texture;
	delete[] nega_texture;

}

/*
** ���e�ʂ�3�������W���i�[����֐�
*/
void myGL::getWorldPoint()
{
	//�v���W�F�N�^�𑜓x���f�[�^���擾
	//�G���[��f�ɂ� -1 ���i�[
	gc.getCorrespondAllPoints(projPoint, imagePoint, pointColor);
	// �Ή��_�̘c�ݏ���
	std::vector<cv::Point2f> undistort_imagePoint;
	std::vector<cv::Point2f> undistort_projPoint;
	cv::undistortPoints(imagePoint, undistort_imagePoint, calib.cam_K, calib.cam_dist);
	cv::undistortPoints(projPoint, undistort_projPoint, calib.proj_K, calib.proj_dist);
	for (int i = 0; i < imagePoint.size(); ++i)
	{
		undistort_imagePoint[i].x = undistort_imagePoint[i].x * calib.cam_K.at<double>(0, 0) + calib.cam_K.at<double>(0, 2);
		undistort_imagePoint[i].y = undistort_imagePoint[i].y * calib.cam_K.at<double>(1, 1) + calib.cam_K.at<double>(1, 2);
		undistort_projPoint[i].x = undistort_projPoint[i].x * calib.proj_K.at<double>(0, 0) + calib.proj_K.at<double>(0, 2);
		undistort_projPoint[i].y = undistort_projPoint[i].y * calib.proj_K.at<double>(1, 1) + calib.proj_K.at<double>(1, 2);
	}

	// 3�������W�̎擾
	calib.reconstruction(reconstructPoint, undistort_projPoint, undistort_imagePoint);

	//����������
	smoothing();
}


/*
** ���e�ʂ̕���������
*/
void myGL::smoothing()
{

}

/*
** user view �̓������e�s����v�Z����֐�
** 3�����č\�����s������ɌĂ΂Ȃ��Ƃ����Ȃ�
** ���[�U�̈ʒu�����擾��ɌĂ΂Ȃ��Ƃ����Ȃ�
*/
cv::Mat myGL::calcUserPerspectiveMatrix(cv::Point3f userPos)
{
	// 3������������Ă��Ȃ��ꍇ�K���Ȓl��Ԃ�
	if (reconstructPoint.size() == 0){
		std::cout << "user_trans is null point" << std::endl;
		cv::Mat n;
		return n;
	}

	// ���[�U�̈ʒu�փJ�������ړ�����
	// ��]�s��(PGR�J�������烆�[�U�ւ̉�]�B���ێg�p����̂̓��[�U����PGR�J�����ւ̉�])
	cv::Mat user_R(3, 1, CV_64F);
	user_R.at<double>(0, 0) = 0;	// roll(x�����̉�])
	user_R.at<double>(1, 0) = 0;	// pich(y�����̉�])
	user_R.at<double>(2, 0) = 0;	// yaw(z�����̉�])
	cv::Rodrigues(user_R, user_R);
	cv::Mat user_R_inv = user_R.t();

	// ���i�s��(userPos��PGR�J�������W�n�ɂ����郆�[�U�̈ʒu�x�N�g���B���ێg�p����̂̓��[�U���W�n�ɂ�����PGR�J�����̈ʒu�x�N�g��
	// �܂���W�ϊ����K�v
	cv::Mat user_T(3, 1, CV_64F);
	user_T.at<double>(0, 0) = userPos.x;	// roll(x�����̉�])
	user_T.at<double>(1, 0) = userPos.y;	// pich(y�����̉�])
	user_T.at<double>(2, 0) = userPos.z;	// yaw(z�����̉�])
	cv::Mat worldPoint(3, 1, CV_64F);
	worldPoint.at<double>(0, 0) = 0.0;
	worldPoint.at<double>(1, 0) = 0.0;
	worldPoint.at<double>(2, 0) = 0.0;
	cv::Mat Tvec = -1.0 * user_R_inv * user_T;

	// ��]�ƕ��i������
	cv::Mat extrinsic(4, 4, CV_64F);
	extrinsic.at<double>(0, 0) = user_R_inv.at<double>(0, 0);
	extrinsic.at<double>(0, 1) = user_R_inv.at<double>(0, 1);
	extrinsic.at<double>(0, 2) = user_R_inv.at<double>(0, 2);
	extrinsic.at<double>(1, 0) = user_R_inv.at<double>(1, 0);
	extrinsic.at<double>(1, 1) = user_R_inv.at<double>(1, 1);
	extrinsic.at<double>(1, 2) = user_R_inv.at<double>(1, 2);
	extrinsic.at<double>(2, 0) = user_R_inv.at<double>(2, 0);
	extrinsic.at<double>(2, 1) = user_R_inv.at<double>(2, 1);
	extrinsic.at<double>(2, 2) = user_R_inv.at<double>(2, 2);
	extrinsic.at<double>(0, 3) = Tvec.at<double>(0, 0);
	extrinsic.at<double>(1, 3) = Tvec.at<double>(1, 0);
	extrinsic.at<double>(2, 3) = Tvec.at<double>(2, 0);
	extrinsic.at<double>(3, 0) = 0.0;
	extrinsic.at<double>(3, 1) = 0.0;
	extrinsic.at<double>(3, 2) = 0.0;
	extrinsic.at<double>(3, 3) = 1.0;

	// �����p�����[�^�̕ό`�i�ϑ��J�����̓����p�����[�^���g�p�j
	cv::Mat intrinsic(3, 4, CV_64F);
	intrinsic.at<double>(0, 0) = calib.cam_K.at<double>(0, 0);
	intrinsic.at<double>(0, 1) = calib.cam_K.at<double>(0, 1);
	intrinsic.at<double>(0, 2) = calib.cam_K.at<double>(0, 2);
	intrinsic.at<double>(1, 0) = calib.cam_K.at<double>(1, 0);
	intrinsic.at<double>(1, 1) = calib.cam_K.at<double>(1, 1);
	intrinsic.at<double>(1, 2) = calib.cam_K.at<double>(1, 2);
	intrinsic.at<double>(2, 0) = calib.cam_K.at<double>(2, 0);
	intrinsic.at<double>(2, 1) = calib.cam_K.at<double>(2, 1);
	intrinsic.at<double>(2, 2) = calib.cam_K.at<double>(2, 2);
	intrinsic.at<double>(0, 3) = 0.0;
	intrinsic.at<double>(1, 3) = 0.0;
	intrinsic.at<double>(2, 3) = 0.0;

	return intrinsic * extrinsic;

}
