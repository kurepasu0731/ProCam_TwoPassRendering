#ifndef CALIBRATION_H
#define CALIBRATION_H
#include <opencv2\opencv.hpp>


class Calibration
{
public:
	Calibration(){};
	~Calibration(){};

	// �L�����u���[�V�������ʂ̓ǂݍ���
	void loadCalibParam(const std::string &fileName);
	void loadPGR2PGRCalibParam(const std::string &fileName); // pgr-pgr�̃L�����u���[�V��������
	void loadPGR2KINECTCalibParam(const std::string &fileName); // PGR-KinectColor�̃L�����u���[�V��������
	void loadKINECT2KINECTCalibParam(const std::string &fileName); // KinectColor-IR�̃L�����u���[�V��������

	// �������e�ϊ��s��̎擾(�J����)
	cv::Mat getCamPerspectiveMat();
	// �������e�ϊ��s��̎擾(�v���W�F�N�^)
	cv::Mat getProjPerspectiveMat();

	// �J�����ʒu�����[���h���W�Ƃ����ۂ̑Ώە��̂̈ʒu�̎擾
	void getCameraWorldPoint(std::vector<cv::Point3f> &camWorldPoint, const std::vector<cv::Point2f> &imagePoint);

	// 3��������
	void reconstruction(std::vector<cv::Point3f> &reconstructPoint, const std::vector<cv::Point2f> &projPoint, const std::vector<cv::Point2f> &imagePoint);

	cv::Mat getHomogeneousCoodinateMatrix(cv::Mat &R, cv::Mat &T);

	// kinect���W�n����PGR�J�������W�n�֕ϊ�����s��
	void getKinectSpaceToCameraSpaceMatrix(cv::Mat &matrix);

	// projector���J�����ւ̉�]���i�s��
	void getCamToProjMatrix(cv::Mat &matrix);
	void getProjToCamMatrix(cv::Mat &matrix);

	/***** �����o�ϐ� *****/
	cv::Size checkerPattern;		// �`�F�b�J�[�p�^�[���̌�_�̐�
	float checkerSize;				// �`�F�b�J�[�p�^�[���̃}�X�ڂ̃T�C�Y(mm)

	std::vector<cv::Point3f> worldPoint;		// �`�F�b�J�[��_���W�ƑΉ����鐢�E���W�̒l���i�[����s��

	// �J����
	cv::Mat cam_K;					// �����p�����[�^�s��
	cv::Mat cam_dist;				// �����Y�c��
	std::vector<cv::Mat> cam_R;		// ��]�x�N�g��
	std::vector<cv::Mat> cam_T;		// ���s�ړ��x�N�g��

	// �v���W�F�N�^
	cv::Mat proj_K;					// �����p�����[�^�s��
	cv::Mat proj_dist;				// �����Y�c��
	std::vector<cv::Mat> proj_R;	// ��]�x�N�g��
	std::vector<cv::Mat> proj_T;	// ���s�ړ��x�N�g��

	// �X�e���I�p�����[�^
	cv::Mat R;						// �J����-�v���W�F�N�^�Ԃ̉�]�s��
	cv::Mat T;						// �J����-�v���W�F�N�^�Ԃ̕��i�x�N�g��
	cv::Mat E;						// ��{�s��
	cv::Mat F;						// ��b�s��
	cv::Mat PGR2PGR_R;
	cv::Mat PGR2PGR_T;
	cv::Mat PGR2KINECT_R;
	cv::Mat PGR2KINECT_T;
	cv::Mat KINECT2KINECT_R;				// PGR�J����-kinect�Ԃ̉�]�s��
	cv::Mat KINECT2KINECT_T;				// PGR�J����-kinect�Ԃ̕��i�x�N�g��


	// �t���O
	bool calib_flag;
};

#endif