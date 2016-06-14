#ifndef TEXTURE_H
#define TEXTURE_H

#pragma once
#include <GL/freeglut.h>
#include <opencv2/opencv.hpp>

class MyTexture{

public:
	MyTexture();
	~MyTexture();

	// �𑜓x���i�[���Ă���
	int imgWidth, imgHeight;
	int movWidth, movHeight;
	// �摜�Ɠ���̃A�h���X
	std::string imgUrl, movUrl;
	// �E�B���h�E�̃t���[���T�C�Y
	int frameWidth, frameHeight;
	// �Ō�Ƀe�N�X�`���Ɏg�����摜�̉𑜓x
	int nowTextureWidth, nowTextureHeight;

	bool loadImage(std::string imgFilename, GLuint &textureImg);
	bool setTexture(cv::Mat &Image, GLuint* TextureName, bool mipmaps = 0);
	bool setFramebufferTexture(GLuint* fb, GLuint* cb, GLuint* rb, int w, int h);
	void deleteFramebufferTexture(GLuint* fb, GLuint* cb, GLuint* rb);
	void TextureRendering();
	void setMultiTexture(cv::Mat &Image, GLuint &TextureName, int TextureUnit, bool mipmaps = 0);	//�}���`�e�N�X�`���𗘗p����ꍇ������
	cv::Mat getImageMat(std::string imgFilename);	// Mat�^�̉摜�f�[�^�����擾����֐��i�e�N�X�`��������O�ɉ������珈�����������Ƃ��Ɏg�p�j
	void projectiveTextureMapping(bool toggle);
	void texture2mat(GLenum MODE, int window_w, int window_h, cv::Mat &matdata);
	void textureRenderer(GLuint &texture);

private:
	// �ˉe�e�N�X�`���}�b�s���O�̃e�N�X�`�����W���������p�z��
	double genfunc[4][4];
	bool loadImage(std::string imgFilename, GLuint &textureImg, bool mipmaps);
	void setTexture(cv::Mat Image, GLuint &textureID, bool mipmaps);
	void setFramebufferTexture(GLuint &framebuf, GLuint &colorbuf, GLuint &depthbuf);
	void fillImageOutline(cv::Mat &src, cv::Mat &dist);
};


//***************************
//	�ȉ��e���\�b�h�̏���
//***************************

// �N���X�쐬���ɂ������ŏ��ɌĂ΂��
inline MyTexture::MyTexture()
{
	//�R���X�g���N�^
}

inline MyTexture::~MyTexture()
{
	// �f�X�g���N�^
}

inline bool MyTexture::loadImage(std::string imgFilename, GLuint &textureImg)
{
	return loadImage(imgFilename, textureImg, 0);
}

inline bool MyTexture::loadImage(std::string imgFilename, GLuint &textureImg, bool mipmaps)
{
	cv::Mat image;

	std::cout << imgFilename << std::endl;
	image = cv::imread(imgFilename, -1);
	if (image.empty())
	{
		std::cerr << "ERROR�F�摜���ǂݍ��߂܂���" << std::endl;
		image = cv::Mat(cv::Size(256, 256), CV_8UC3, cv::Scalar::all(0));
		return false;
	}
	// �t�@�C�����Ɖ𑜓x��ۊǂ��Ă���
	imgUrl = imgFilename;
	imgWidth = image.cols;
	imgHeight = image.rows;

	// �摜�̎���1pixel��h��Ԃ�
	fillImageOutline(image, image);

	// �e�N�X�`����
	setTexture(image, textureImg, mipmaps);

	return true;
}

/*
** �F�X�摜�������Ă���e�N�X�`���Ɏg�p����o�[�W����
** mipmaps���g���Ƌɒ[�ɏd���Ȃ�̂ŁA�摜�ȊO�ł͔񐄏�
*/
inline bool MyTexture::setTexture(cv::Mat &Image, GLuint* textureID, bool mipmaps){
	// �ǂݍ��߂Ȃ������ꍇ�͐^�����̉摜�ɂ���
	if (Image.data == NULL)
	{
		std::cerr << "ERROR�F�摜���ǂݍ��߂܂���" << std::endl;
		Image = cv::Mat(cv::Size(256, 256), CV_8UC3, cv::Scalar::all(255));
		return false;
	}
	setTexture(Image, *textureID, mipmaps);
	return true;
}

inline 	void MyTexture::setTexture(cv::Mat Image, GLuint &textureID, bool mipmaps)
{

	nowTextureWidth = Image.cols;
	nowTextureHeight = Image.rows;
	int format = GL_RGBA;

	// (�摜��*�`�����l����)%4 != 0 ����Mat�^�Ő������ǂݍ��܂�Ȃ��̂ŁA�ǂ̏ꍇ��RGBA�ɕϊ����ĕK������؂��悤�ɂ���
	if (Image.channels() == 1)
		cv::cvtColor(Image, Image, CV_GRAY2RGBA);
	else if (Image.channels() == 4)
		cv::cvtColor(Image, Image, CV_BGRA2RGBA);
	else
		cv::cvtColor(Image, Image, CV_BGR2RGBA);

	//cv::flip(Image, Image, 0);
	glGenTextures(1, &textureID);
	glBindTexture(GL_TEXTURE_2D, textureID); //�w�肵�����O�̃e�N�X�`����L����
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, nowTextureWidth, nowTextureHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, Image.data);
	//if (mipmaps == 1)
	//gluBuild2DMipmaps(GL_TEXTURE_2D, 3, nowTextureWidth, nowTextureHeight, GL_RGBA, GL_UNSIGNED_BYTE, Image.data);

	/* �e�N�X�`�����g��E�k��������@�̎w�� */
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	/* �e�N�X�`���̌J��Ԃ��̎w�� */
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

	/* �e�N�X�`���� */
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);	//�����l:GL_MODULATE�F�e�N�X�`���ގ����|���S���ƍ��킹�邩�������邩�����肷��,GL_REPLACE

	//�o�C���h������
	glBindTexture(GL_TEXTURE_2D, 0);
}

inline 	void MyTexture::setMultiTexture(cv::Mat &Image, GLuint &textureID, int TextureUnit, bool mipmaps)
{

	nowTextureWidth = Image.cols;
	nowTextureHeight = Image.rows;
	int format = GL_RGBA;

	// (�摜��*�`�����l����)%4 != 0 ����Mat�^�Ő������ǂݍ��܂�Ȃ��̂ŁA�ǂ̏ꍇ��RGBA�ɕϊ����ĕK������؂��悤�ɂ���
	if (Image.channels() == 1)
		cv::cvtColor(Image, Image, CV_GRAY2RGBA);
	else if (Image.channels() == 4)
		cv::cvtColor(Image, Image, CV_BGRA2RGBA);
	else
		cv::cvtColor(Image, Image, CV_BGR2RGBA);

	//cv::flip(Image, Image, 0);
	glGenTextures(1, &textureID);
	glBindTexture(GL_TEXTURE_2D, textureID); //�w�肵�����O�̃e�N�X�`����L����
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, nowTextureWidth, nowTextureHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, Image.data);
	//if (mipmaps == 1)
	//gluBuild2DMipmaps(GL_TEXTURE_2D, 3, nowTextureWidth, nowTextureHeight, GL_RGBA, GL_UNSIGNED_BYTE, Image.data);

	/* �e�N�X�`�����g��E�k��������@�̎w�� */
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	/* �e�N�X�`���̌J��Ԃ��̎w�� */
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

	/* �e�N�X�`���� */
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);	//�����l:GL_MODULATE�F�e�N�X�`���ގ����|���S���ƍ��킹�邩�������邩�����肷��

	//�o�C���h������
	//glBindTexture(GL_TEXTURE_2D, 0);
}

inline bool MyTexture::setFramebufferTexture(GLuint* fb, GLuint* cb, GLuint* rb, int w, int h)
{
	int FBOWIDTH = w;
	int FBOHEIGHT = h;

	// �J���[�o�b�t�@�p�̃e�N�X�`����p�ӂ���
	glGenTextures(1, cb);	//�e�N�X�`���I�u�W�F�N�g�̐���
	glBindTexture(GL_TEXTURE_2D, *cb);	//�w�肵�����O�̃e�N�X�`����L����
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, FBOWIDTH, FBOHEIGHT, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);	//�摜�f�[�^�̊֘A�t��
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glBindTexture(GL_TEXTURE_2D, 0);

	// �f�v�X�o�b�t�@�p�̃����_�[�o�b�t�@��p�ӂ���
	glGenRenderbuffersEXT(1, rb);
	glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, *rb);
	glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT, FBOWIDTH, FBOHEIGHT);
	glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, 0);

	// �t���[���o�b�t�@�I�u�W�F�N�g���쐬����
	glGenFramebuffersEXT(1, fb);
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, *fb);

	// �t���[���o�b�t�@�I�u�W�F�N�g�ɃJ���[�o�b�t�@�Ƃ��ăe�N�X�`������������
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, *cb, 0);

	// �t���[���o�b�t�@�I�u�W�F�N�g�Ƀf�v�X�o�b�t�@�Ƃ��ă����_�[�o�b�t�@����������
	glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, *rb);

	// �t���[���o�b�t�@�I�u�W�F�N�g�̌�������������
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);

	return true;
}

inline void MyTexture::deleteFramebufferTexture(GLuint* fb, GLuint* cb, GLuint* rb)
{
	glDeleteTextures(1, cb);
	glDeleteRenderbuffersEXT(1, rb);
	glDeleteFramebuffersEXT(1, fb);
}

inline void MyTexture::setFramebufferTexture(GLuint &framebuf, GLuint &colorbuf, GLuint &depthbuf)
{

}
/*
** Mat�^�̉摜�f�[�^���擾����֐�
** imgUrl, imgWidth, imgHeight�ɂ��ꂼ��A�h���X, ��, �����������Ă���
** �e�N�X�`�����������Ƃ���setTextureEx�ɓn��
*/
inline cv::Mat MyTexture::getImageMat(std::string imgFilename){
	cv::Mat image;

	image = cv::imread(imgFilename, -1);
	if (image.data == NULL)	{
		std::cerr << "ERROR�F�摜���ǂݍ��߂܂���" << std::endl;
		image = cv::Mat(cv::Size(256, 256), CV_8UC3, cv::Scalar::all(0));
	}
	// �t�@�C�����Ɖ𑜓x��ۊǂ��Ă���
	imgUrl = imgFilename;
	imgWidth = image.cols;
	imgHeight = image.rows;
	return image;
}

inline void MyTexture::fillImageOutline(cv::Mat &src, cv::Mat &dist)
{
	int height = src.rows;
	int width = src.cols;
	for (int y = 0; y<height; y++){
		for (int x = 0; x<width; x++){
			if (y == 0 || y == (height - 1) || x == 0 || x == (width - 1)){
				src.at<uchar>(y, 3 * x) = 255;
				src.at<uchar>(y, 3 * x + 1) = 255;
				src.at<uchar>(y, 3 * x + 2) = 255;
			}
		}
	}

}
/*
** ���e�e�N�X�`���}�b�s���O���s���悤�ɂ���֐�
** ���e�e�N�X�`���}�b�s���O�s���I�u�W�F�N�g��`�悷��֐������̊֐���true��false�ŋ���
*/
inline void MyTexture::projectiveTextureMapping(bool toggle){
	if (toggle){
		// �e�N�X�`�����W���������p�̍s����쐬
		static double _genfunc[][4] = {
			{ 1.0, 0.0, 0.0, 0.0 },
			{ 0.0, 1.0, 0.0, 0.0 },
			{ 0.0, 0.0, 1.0, 0.0 },
			{ 0.0, 0.0, 0.0, 1.0 },
		};
		for (int i = 0; i<4; i++){
			for (int j = 0; j<4; j++){
				genfunc[i][j] = _genfunc[i][j];
			}
		}

		// �e�N�X�`���Əd�Ȃ�悤���s�ړ�
		glMatrixMode(GL_TEXTURE);
		glLoadIdentity();
		glTranslated(0.5, 0.5, 0.0);
		glScaled(1.0, 1.0, 1.0);

		// ���_�̃I�u�W�F�N�g��Ԃɂ�������W�l���g���ă}�b�s���O����
		glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
		glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
		glTexGeni(GL_R, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
		glTexGeni(GL_Q, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);

		// �e�N�X�`�����W�����֐��̐ݒ�
		glTexGendv(GL_S, GL_OBJECT_PLANE, genfunc[0]);
		glTexGendv(GL_T, GL_OBJECT_PLANE, genfunc[1]);
		glTexGendv(GL_R, GL_OBJECT_PLANE, genfunc[2]);
		glTexGendv(GL_Q, GL_OBJECT_PLANE, genfunc[3]);

		// �e�N�X�`���}�b�s���O�J�n
		glEnable(GL_TEXTURE_2D);
		// �e�N�X�`�����W�̎���������L���ɂ���
		glEnable(GL_TEXTURE_GEN_S);
		glEnable(GL_TEXTURE_GEN_T);
		glEnable(GL_TEXTURE_GEN_R);
		glEnable(GL_TEXTURE_GEN_Q);
	}
	else{
		// �e�N�X�`�����W�̎��������𖳌��ɂ���
		glDisable(GL_TEXTURE_GEN_S);
		glDisable(GL_TEXTURE_GEN_T);
		glDisable(GL_TEXTURE_GEN_R);
		glDisable(GL_TEXTURE_GEN_Q);
		// �e�N�X�`���}�b�s���O�I��
		glDisable(GL_TEXTURE_2D);
	}
}
//inline void MyTexture::projectiveTextureMapping(bool toggle){
//	if (toggle){
//		// �e�N�X�`�����W���������p�̍s����쐬
//		static double _genfunc[][4] = {
//			{ 1.0, 0.0, 0.0, 0.0 },
//			{ 0.0, 1.0, 0.0, 0.0 },
//			{ 0.0, 0.0, 1.0, 0.0 },
//			{ 0.0, 0.0, 0.0, 1.0 },
//		};
//		for (int i = 0; i<4; i++){
//			for (int j = 0; j<4; j++){
//				genfunc[i][j] = _genfunc[i][j];
//			}
//		}
//
//		// �e�N�X�`���Əd�Ȃ�悤���s�ړ�
//		glMatrixMode(GL_TEXTURE);
//		glLoadIdentity();
//		glTranslated(0.5, 0.5, 0.0);
//		glScaled(1.0, 1.0, 1.0);
//
//		// ���_�̃I�u�W�F�N�g��Ԃɂ�������W�l���g���ă}�b�s���O����
//		glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
//		glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
//		glTexGeni(GL_R, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
//		glTexGeni(GL_Q, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
//
//		// �e�N�X�`�����W�����֐��̐ݒ�
//		glTexGendv(GL_S, GL_OBJECT_PLANE, genfunc[0]);
//		glTexGendv(GL_T, GL_OBJECT_PLANE, genfunc[1]);
//		glTexGendv(GL_R, GL_OBJECT_PLANE, genfunc[2]);
//		glTexGendv(GL_Q, GL_OBJECT_PLANE, genfunc[3]);
//
//		// �e�N�X�`���}�b�s���O�J�n
//		glEnable(GL_TEXTURE_2D);
//		// �e�N�X�`�����W�̎���������L���ɂ���
//		glEnable(GL_TEXTURE_GEN_S);
//		glEnable(GL_TEXTURE_GEN_T);
//		glEnable(GL_TEXTURE_GEN_R);
//		glEnable(GL_TEXTURE_GEN_Q);
//	}
//	else{
//		// �e�N�X�`�����W�̎��������𖳌��ɂ���
//		glDisable(GL_TEXTURE_GEN_S);
//		glDisable(GL_TEXTURE_GEN_T);
//		glDisable(GL_TEXTURE_GEN_R);
//		glDisable(GL_TEXTURE_GEN_Q);
//		// �e�N�X�`���}�b�s���O�I��
//		glDisable(GL_TEXTURE_2D);
//	}
//}


/*
** �e�N�X�`����Mat������֐�
*/
inline void MyTexture::texture2mat(GLenum MODE, int window_w, int window_h, cv::Mat &matdata)
{
	// �e�N�X�`�����摜��
	//Texture��GL_COLOR_ATTACHMENT0_EXT�Ɋ��蓖�Ă��Ă���Ƃ���
	glReadBuffer(MODE);// FBO : GL_COLOR_ATTACHMENT0_EXT, GL_FRONT, GL_BACK
	glReadPixels(0, 0, window_w, window_h, GL_BGRA, GL_UNSIGNED_BYTE, (void*)matdata.data);
	//cv::flip(matdata, matdata, 0);

}

/*
** �v���p�̉摜�𓊉e����Ƃ��Ɏg�p���郌���_�����O�֐�
** �֐��Ăяo���O�ɕϊ��s��������l�ɃZ�b�g���Ă�������
*/
inline void MyTexture::textureRenderer(GLuint &texture)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	//�o�b�t�@�N���A

	// �A�e�t���ƉB�ʏ��������͍s��Ȃ�
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);

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

	glutSwapBuffers();

}

#endif