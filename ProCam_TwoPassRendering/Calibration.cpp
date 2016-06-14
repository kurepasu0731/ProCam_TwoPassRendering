#include "Calibration.h"


// �L�����u���[�V�������ʂ̓ǂݍ���
void Calibration::loadCalibParam(const std::string &fileName)
{
	// xml�t�@�C���̓ǂݍ���
	cv::FileStorage cvfs(fileName, cv::FileStorage::READ);

	cvfs["cam_K"] >> cam_K;
	cvfs["cam_dist"] >> cam_dist;
	cvfs["cam_R"] >> cam_R;
	cvfs["cam_T"] >> cam_T;
	cvfs["proj_K"] >> proj_K;
	cvfs["proj_dist"] >> proj_dist;
	cvfs["proj_R"] >> proj_R;
	cvfs["proj_T"] >> proj_T;
	cvfs["R"] >> R;
	cvfs["T"] >> T;
	cvfs["E"] >> E;
	cvfs["F"] >> F;

	calib_flag = true;

}

void Calibration::loadPGR2PGRCalibParam(const std::string &fileName)
{
	// xml�t�@�C���̓ǂݍ���
	cv::FileStorage cvfs(fileName, cv::FileStorage::READ);

	// kinect pgrcamera ����
	cvfs["R"] >> PGR2PGR_R;
	cvfs["T"] >> PGR2PGR_T;
}

void Calibration::loadPGR2KINECTCalibParam(const std::string &fileName)
{
	// xml�t�@�C���̓ǂݍ���
	cv::FileStorage cvfs(fileName, cv::FileStorage::READ);

	// kinect pgrcamera ����
	cvfs["R"] >> PGR2KINECT_R;
	cvfs["T"] >> PGR2KINECT_T;
}
void Calibration::loadKINECT2KINECTCalibParam(const std::string &fileName)
{
	// xml�t�@�C���̓ǂݍ���
	cv::FileStorage cvfs(fileName, cv::FileStorage::READ);

	// kinect pgrcamera ����
	cvfs["R"] >> KINECT2KINECT_R;
	cvfs["T"] >> KINECT2KINECT_T;
}

// �������e�ϊ��s��̎擾(�J����)
cv::Mat Calibration::getCamPerspectiveMat()
{
	// ��]�ƕ��i������
	cv::Mat extrinsic = cv::Mat::eye(4, 4, CV_64F);

	// �����p�����[�^�̕ό`
	cv::Mat intrinsic(3, 4, CV_64F);
	intrinsic.at<double>(0, 0) = cam_K.at<double>(0, 0);
	intrinsic.at<double>(0, 1) = cam_K.at<double>(0, 1);
	intrinsic.at<double>(0, 2) = cam_K.at<double>(0, 2);
	intrinsic.at<double>(1, 0) = cam_K.at<double>(1, 0);
	intrinsic.at<double>(1, 1) = cam_K.at<double>(1, 1);
	intrinsic.at<double>(1, 2) = cam_K.at<double>(1, 2);
	intrinsic.at<double>(2, 0) = cam_K.at<double>(2, 0);
	intrinsic.at<double>(2, 1) = cam_K.at<double>(2, 1);
	intrinsic.at<double>(2, 2) = cam_K.at<double>(2, 2);
	intrinsic.at<double>(0, 3) = 0.0;
	intrinsic.at<double>(1, 3) = 0.0;
	intrinsic.at<double>(2, 3) = 0.0;

	return intrinsic * extrinsic;
}


// �������e�ϊ��s��̎擾(�v���W�F�N�^)
cv::Mat Calibration::getProjPerspectiveMat()
{
	// ��]�ƕ��i������
	cv::Mat extrinsic(4, 4, CV_64F);
	extrinsic.at<double>(0, 0) = R.at<double>(0, 0);
	extrinsic.at<double>(0, 1) = R.at<double>(0, 1);
	extrinsic.at<double>(0, 2) = R.at<double>(0, 2);
	extrinsic.at<double>(1, 0) = R.at<double>(1, 0);
	extrinsic.at<double>(1, 1) = R.at<double>(1, 1);
	extrinsic.at<double>(1, 2) = R.at<double>(1, 2);
	extrinsic.at<double>(2, 0) = R.at<double>(2, 0);
	extrinsic.at<double>(2, 1) = R.at<double>(2, 1);
	extrinsic.at<double>(2, 2) = R.at<double>(2, 2);
	extrinsic.at<double>(0, 3) = T.at<double>(0, 0);
	extrinsic.at<double>(1, 3) = T.at<double>(1, 0);
	extrinsic.at<double>(2, 3) = T.at<double>(2, 0);
	extrinsic.at<double>(3, 0) = 0.0;
	extrinsic.at<double>(3, 1) = 0.0;
	extrinsic.at<double>(3, 2) = 0.0;
	extrinsic.at<double>(3, 3) = 1.0;

	// �����p�����[�^�̕ό`
	cv::Mat intrinsic(3, 4, CV_64F);
	intrinsic.at<double>(0, 0) = proj_K.at<double>(0, 0);
	intrinsic.at<double>(0, 1) = proj_K.at<double>(0, 1);
	intrinsic.at<double>(0, 2) = proj_K.at<double>(0, 2);
	intrinsic.at<double>(1, 0) = proj_K.at<double>(1, 0);
	intrinsic.at<double>(1, 1) = proj_K.at<double>(1, 1);
	intrinsic.at<double>(1, 2) = proj_K.at<double>(1, 2);
	intrinsic.at<double>(2, 0) = proj_K.at<double>(2, 0);
	intrinsic.at<double>(2, 1) = proj_K.at<double>(2, 1);
	intrinsic.at<double>(2, 2) = proj_K.at<double>(2, 2);
	intrinsic.at<double>(0, 3) = 0.0;
	intrinsic.at<double>(1, 3) = 0.0;
	intrinsic.at<double>(2, 3) = 0.0;

	return intrinsic * extrinsic;
}


// �J�����ʒu�����[���h���W�Ƃ����ۂ̑Ώە��̂̈ʒu�̎擾
void Calibration::getCameraWorldPoint(std::vector<cv::Point3f> &camWorldPoint, const std::vector<cv::Point2f> &imagePoint)
{
	cv::Mat rvec, tvec, rmat;

	// �`�F�b�J�[�p�^�[���̈ʒu���o
	cv::solvePnP(worldPoint, imagePoint, cam_K, cv::Mat(), rvec, tvec);

	cv::Rodrigues(rvec, rmat);		// ��]�s��ɕϊ�

	// �`�F�b�J�[�p�^�[�����S����J�������S�ɍ��W�ϊ�
	rmat = rmat.t();	// �]�u�s��

	cv::Mat extrinsic(4, 4, CV_64F);
	extrinsic.at<double>(0, 0) = rmat.at<double>(0, 0);
	extrinsic.at<double>(0, 1) = rmat.at<double>(0, 1);
	extrinsic.at<double>(0, 2) = rmat.at<double>(0, 2);
	extrinsic.at<double>(1, 0) = rmat.at<double>(1, 0);
	extrinsic.at<double>(1, 1) = rmat.at<double>(1, 1);
	extrinsic.at<double>(1, 2) = rmat.at<double>(1, 2);
	extrinsic.at<double>(2, 0) = rmat.at<double>(2, 0);
	extrinsic.at<double>(2, 1) = rmat.at<double>(2, 1);
	extrinsic.at<double>(2, 2) = rmat.at<double>(2, 2);
	extrinsic.at<double>(0, 3) = cv::Mat(-rmat*tvec).at<double>(0, 0);
	extrinsic.at<double>(1, 3) = cv::Mat(-rmat*tvec).at<double>(1, 0);
	extrinsic.at<double>(2, 3) = cv::Mat(-rmat*tvec).at<double>(2, 0);
	extrinsic.at<double>(3, 0) = 0.0;
	extrinsic.at<double>(3, 1) = 0.0;
	extrinsic.at<double>(3, 2) = 0.0;
	extrinsic.at<double>(3, 3) = 1.0;

	// �`�F�b�J�[�p�^�[���̌�_�ʒu
	for (int i = 0; i<worldPoint.size(); ++i)
	{
		cv::Mat checker_pos = extrinsic.inv() * cv::Mat((cv::Mat_<double>(4, 1) << worldPoint[i].x, worldPoint[i].y, worldPoint[i].z, 1.0));		// �`�F�b�J�[�p�^�[���̈ʒu
		camWorldPoint.emplace_back(cv::Point3f(checker_pos.at<double>(0) / checker_pos.at<double>(3), checker_pos.at<double>(1) / checker_pos.at<double>(3), checker_pos.at<double>(2) / checker_pos.at<double>(3)));
	}
}


// 3��������
void Calibration::reconstruction(std::vector<cv::Point3f> &reconstructPoint, const std::vector<cv::Point2f> &projPoint, const std::vector<cv::Point2f> &imagePoint)
{
	// �������e�ϊ��s��
	cv::Mat cam_pers = getCamPerspectiveMat();
	cv::Mat proj_pers = getProjPerspectiveMat();


	static cv::Mat f(4, 1, CV_64FC1, cv::Scalar(0.0));
	static cv::Mat q(4, 3, CV_64FC1, cv::Scalar(0.0));
	static cv::Mat v(3, 1, CV_64FC1, cv::Scalar(0.0));

	// object space�̍ŏ����ɂ��3��������
	//for(int i=0; i<projPoint.size(); ++i) //original
	for (int i = 0; i<imagePoint.size(); ++i)
	{
		f.at<double>(0, 0) = imagePoint[i].x * cam_pers.at<double>(2, 3) - cam_pers.at<double>(0, 3);
		f.at<double>(1, 0) = imagePoint[i].y * cam_pers.at<double>(2, 3) - cam_pers.at<double>(1, 3);
		f.at<double>(2, 0) = projPoint[i].x * proj_pers.at<double>(2, 3) - proj_pers.at<double>(0, 3);
		f.at<double>(3, 0) = projPoint[i].y * proj_pers.at<double>(2, 3) - proj_pers.at<double>(1, 3);

		q.at<double>(0, 0) = cam_pers.at<double>(0, 0) - cam_pers.at<double>(2, 0) * imagePoint[i].x;
		q.at<double>(0, 1) = cam_pers.at<double>(0, 1) - cam_pers.at<double>(2, 1) * imagePoint[i].x;
		q.at<double>(0, 2) = cam_pers.at<double>(0, 2) - cam_pers.at<double>(2, 2) * imagePoint[i].x;
		q.at<double>(1, 0) = cam_pers.at<double>(1, 0) - cam_pers.at<double>(2, 0) * imagePoint[i].y;
		q.at<double>(1, 1) = cam_pers.at<double>(1, 1) - cam_pers.at<double>(2, 1) * imagePoint[i].y;
		q.at<double>(1, 2) = cam_pers.at<double>(1, 2) - cam_pers.at<double>(2, 2) * imagePoint[i].y;
		q.at<double>(2, 0) = proj_pers.at<double>(0, 0) - proj_pers.at<double>(2, 0) * projPoint[i].x;
		q.at<double>(2, 1) = proj_pers.at<double>(0, 1) - proj_pers.at<double>(2, 1) * projPoint[i].x;
		q.at<double>(2, 2) = proj_pers.at<double>(0, 2) - proj_pers.at<double>(2, 2) * projPoint[i].x;
		q.at<double>(3, 0) = proj_pers.at<double>(1, 0) - proj_pers.at<double>(2, 0) * projPoint[i].y;
		q.at<double>(3, 1) = proj_pers.at<double>(1, 1) - proj_pers.at<double>(2, 1) * projPoint[i].y;
		q.at<double>(3, 2) = proj_pers.at<double>(1, 2) - proj_pers.at<double>(2, 2) * projPoint[i].y;

		v = q.inv(cv::DECOMP_SVD) * f;
		//v *= 0.001f;
		reconstructPoint.emplace_back(cv::Point3f(v.at<double>(0, 0), v.at<double>(1, 0), v.at<double>(2, 0)));
	}
}

// �O���p�����[�^�𓯎����W�̍s��ŕ\��
cv::Mat Calibration::getHomogeneousCoodinateMatrix(cv::Mat &R, cv::Mat &T)
{
	// �J�����𒆐S�Ƃ����e�f�o�C�X�ւ̉�]�E���i�x�N�g�������߂����Ƃ��͊O���p�����[�^���t�ɂ���K�v������
	cv::Mat rmat = R.t();
	cv::Mat tvec = T;
	cv::Mat extrinsic(4, 4, CV_64F);
	extrinsic.at<double>(0, 0) = rmat.at<double>(0, 0);	// ��]
	extrinsic.at<double>(0, 1) = rmat.at<double>(0, 1);
	extrinsic.at<double>(0, 2) = rmat.at<double>(0, 2);
	extrinsic.at<double>(1, 0) = rmat.at<double>(1, 0);
	extrinsic.at<double>(1, 1) = rmat.at<double>(1, 1);
	extrinsic.at<double>(1, 2) = rmat.at<double>(1, 2);
	extrinsic.at<double>(2, 0) = rmat.at<double>(2, 0);
	extrinsic.at<double>(2, 1) = rmat.at<double>(2, 1);
	extrinsic.at<double>(2, 2) = rmat.at<double>(2, 2);
	extrinsic.at<double>(0, 3) = cv::Mat(-rmat*tvec).at<double>(0, 0);	// ���i
	extrinsic.at<double>(1, 3) = cv::Mat(-rmat*tvec).at<double>(1, 0);
	extrinsic.at<double>(2, 3) = cv::Mat(-rmat*tvec).at<double>(2, 0);
	extrinsic.at<double>(3, 0) = 0.0;
	extrinsic.at<double>(3, 1) = 0.0;
	extrinsic.at<double>(3, 2) = 0.0;
	extrinsic.at<double>(3, 3) = 1.0;
	return extrinsic;

}

// Kinect��PGR�J�������W�n�֕ϊ�����̂ɕK�v�ȍs����v�Z
void Calibration::getKinectSpaceToCameraSpaceMatrix(cv::Mat &matrix)
{
	cv::Mat KIENCT2KINECT_Mat = getHomogeneousCoodinateMatrix(KINECT2KINECT_R, KINECT2KINECT_T);	//KINECT�J���[�J��������IR�J�����ւ̊O���p�����[�^�s��
	cv::Mat PGR2KINECT_Mat = getHomogeneousCoodinateMatrix(PGR2KINECT_R, PGR2KINECT_T);	// PGR�J��������KINECTCOLOR�J�����ւ̊O���p�����[�^�s��

	// KINECTColor-IR�̉�]���i�s��CPGR-KINECT�̉�]���i�s��̏��ɂ����Ă���
	matrix = cv::Mat(4, 4, CV_64F);
	matrix = PGR2KINECT_Mat * KIENCT2KINECT_Mat;

}

void Calibration::getCamToProjMatrix(cv::Mat &matrix)
{
	matrix = cv::Mat(4, 4, CV_64F);
	matrix = getHomogeneousCoodinateMatrix(R, T);

}
void Calibration::getProjToCamMatrix(cv::Mat &matrix)
{
	matrix = cv::Mat(4, 4, CV_64F);
	matrix.at<double>(0, 0) = R.at<double>(0, 0);	// ��]
	matrix.at<double>(0, 1) = R.at<double>(0, 1);
	matrix.at<double>(0, 2) = R.at<double>(0, 2);
	matrix.at<double>(1, 0) = R.at<double>(1, 0);
	matrix.at<double>(1, 1) = R.at<double>(1, 1);
	matrix.at<double>(1, 2) = R.at<double>(1, 2);
	matrix.at<double>(2, 0) = R.at<double>(2, 0);
	matrix.at<double>(2, 1) = R.at<double>(2, 1);
	matrix.at<double>(2, 2) = R.at<double>(2, 2);
	matrix.at<double>(0, 3) = T.at<double>(0, 0);	// ���i
	matrix.at<double>(1, 3) = T.at<double>(1, 0);
	matrix.at<double>(2, 3) = T.at<double>(2, 0);
	matrix.at<double>(3, 0) = 0.0;
	matrix.at<double>(3, 1) = 0.0;
	matrix.at<double>(3, 2) = 0.0;
	matrix.at<double>(3, 3) = 1.0;
}


