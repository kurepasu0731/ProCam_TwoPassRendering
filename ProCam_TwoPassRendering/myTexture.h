#ifndef TEXTURE_H
#define TEXTURE_H

#pragma once
#include <GL/freeglut.h>
#include <opencv2/opencv.hpp>

class MyTexture{

public:
	MyTexture();
	~MyTexture();

	// 解像度を格納しておく
	int imgWidth, imgHeight;
	int movWidth, movHeight;
	// 画像と動画のアドレス
	std::string imgUrl, movUrl;
	// ウィンドウのフレームサイズ
	int frameWidth, frameHeight;
	// 最後にテクスチャに使った画像の解像度
	int nowTextureWidth, nowTextureHeight;

	bool loadImage(std::string imgFilename, GLuint &textureImg);
	bool setTexture(cv::Mat &Image, GLuint* TextureName, bool mipmaps = 0);
	bool setFramebufferTexture(GLuint* fb, GLuint* cb, GLuint* rb, int w, int h);
	void deleteFramebufferTexture(GLuint* fb, GLuint* cb, GLuint* rb);
	void TextureRendering();
	void setMultiTexture(cv::Mat &Image, GLuint &TextureName, int TextureUnit, bool mipmaps = 0);	//マルチテクスチャを利用する場合こっち
	cv::Mat getImageMat(std::string imgFilename);	// Mat型の画像データだけ取得する関数（テクスチャ化する前に何かしら処理加えたいときに使用）
	void projectiveTextureMapping(bool toggle);
	void texture2mat(GLenum MODE, int window_w, int window_h, cv::Mat &matdata);
	void textureRenderer(GLuint &texture);

private:
	// 射影テクスチャマッピングのテクスチャ座標自動生成用配列
	double genfunc[4][4];
	bool loadImage(std::string imgFilename, GLuint &textureImg, bool mipmaps);
	void setTexture(cv::Mat Image, GLuint &textureID, bool mipmaps);
	void setFramebufferTexture(GLuint &framebuf, GLuint &colorbuf, GLuint &depthbuf);
	void fillImageOutline(cv::Mat &src, cv::Mat &dist);
};


//***************************
//	以下各メソッドの処理
//***************************

// クラス作成時にここが最初に呼ばれる
inline MyTexture::MyTexture()
{
	//コンストラクタ
}

inline MyTexture::~MyTexture()
{
	// デストラクタ
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
		std::cerr << "ERROR：画像が読み込めません" << std::endl;
		image = cv::Mat(cv::Size(256, 256), CV_8UC3, cv::Scalar::all(0));
		return false;
	}
	// ファイル名と解像度を保管しておく
	imgUrl = imgFilename;
	imgWidth = image.cols;
	imgHeight = image.rows;

	// 画像の周囲1pixelを塗りつぶす
	fillImageOutline(image, image);

	// テクスチャ化
	setTexture(image, textureImg, mipmaps);

	return true;
}

/*
** 色々画像処理してからテクスチャに使用するバージョン
** mipmapsを使うと極端に重くなるので、画像以外では非推奨
*/
inline bool MyTexture::setTexture(cv::Mat &Image, GLuint* textureID, bool mipmaps){
	// 読み込めなかった場合は真っ白の画像にする
	if (Image.data == NULL)
	{
		std::cerr << "ERROR：画像が読み込めません" << std::endl;
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

	// (画像幅*チャンネル数)%4 != 0 だとMat型で正しく読み込まれないので、どの場合もRGBAに変換して必ず割り切れるようにする
	if (Image.channels() == 1)
		cv::cvtColor(Image, Image, CV_GRAY2RGBA);
	else if (Image.channels() == 4)
		cv::cvtColor(Image, Image, CV_BGRA2RGBA);
	else
		cv::cvtColor(Image, Image, CV_BGR2RGBA);

	//cv::flip(Image, Image, 0);
	glGenTextures(1, &textureID);
	glBindTexture(GL_TEXTURE_2D, textureID); //指定した名前のテクスチャを有効化
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, nowTextureWidth, nowTextureHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, Image.data);
	//if (mipmaps == 1)
	//gluBuild2DMipmaps(GL_TEXTURE_2D, 3, nowTextureWidth, nowTextureHeight, GL_RGBA, GL_UNSIGNED_BYTE, Image.data);

	/* テクスチャを拡大・縮小する方法の指定 */
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	/* テクスチャの繰り返しの指定 */
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

	/* テクスチャ環境 */
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);	//初期値:GL_MODULATE：テクスチャ材質をポリゴンと合わせるか分離するかを決定する,GL_REPLACE

	//バインドを解除
	glBindTexture(GL_TEXTURE_2D, 0);
}

inline 	void MyTexture::setMultiTexture(cv::Mat &Image, GLuint &textureID, int TextureUnit, bool mipmaps)
{

	nowTextureWidth = Image.cols;
	nowTextureHeight = Image.rows;
	int format = GL_RGBA;

	// (画像幅*チャンネル数)%4 != 0 だとMat型で正しく読み込まれないので、どの場合もRGBAに変換して必ず割り切れるようにする
	if (Image.channels() == 1)
		cv::cvtColor(Image, Image, CV_GRAY2RGBA);
	else if (Image.channels() == 4)
		cv::cvtColor(Image, Image, CV_BGRA2RGBA);
	else
		cv::cvtColor(Image, Image, CV_BGR2RGBA);

	//cv::flip(Image, Image, 0);
	glGenTextures(1, &textureID);
	glBindTexture(GL_TEXTURE_2D, textureID); //指定した名前のテクスチャを有効化
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, nowTextureWidth, nowTextureHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, Image.data);
	//if (mipmaps == 1)
	//gluBuild2DMipmaps(GL_TEXTURE_2D, 3, nowTextureWidth, nowTextureHeight, GL_RGBA, GL_UNSIGNED_BYTE, Image.data);

	/* テクスチャを拡大・縮小する方法の指定 */
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	/* テクスチャの繰り返しの指定 */
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

	/* テクスチャ環境 */
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);	//初期値:GL_MODULATE：テクスチャ材質をポリゴンと合わせるか分離するかを決定する

	//バインドを解除
	//glBindTexture(GL_TEXTURE_2D, 0);
}

inline bool MyTexture::setFramebufferTexture(GLuint* fb, GLuint* cb, GLuint* rb, int w, int h)
{
	int FBOWIDTH = w;
	int FBOHEIGHT = h;

	// カラーバッファ用のテクスチャを用意する
	glGenTextures(1, cb);	//テクスチャオブジェクトの生成
	glBindTexture(GL_TEXTURE_2D, *cb);	//指定した名前のテクスチャを有効化
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, FBOWIDTH, FBOHEIGHT, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);	//画像データの関連付け
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glBindTexture(GL_TEXTURE_2D, 0);

	// デプスバッファ用のレンダーバッファを用意する
	glGenRenderbuffersEXT(1, rb);
	glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, *rb);
	glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT, FBOWIDTH, FBOHEIGHT);
	glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, 0);

	// フレームバッファオブジェクトを作成する
	glGenFramebuffersEXT(1, fb);
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, *fb);

	// フレームバッファオブジェクトにカラーバッファとしてテクスチャを結合する
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, *cb, 0);

	// フレームバッファオブジェクトにデプスバッファとしてレンダーバッファを結合する
	glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, *rb);

	// フレームバッファオブジェクトの結合を解除する
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
** Mat型の画像データを取得する関数
** imgUrl, imgWidth, imgHeightにそれぞれアドレス, 幅, 高さが入っている
** テクスチャ化したいときはsetTextureExに渡す
*/
inline cv::Mat MyTexture::getImageMat(std::string imgFilename){
	cv::Mat image;

	image = cv::imread(imgFilename, -1);
	if (image.data == NULL)	{
		std::cerr << "ERROR：画像が読み込めません" << std::endl;
		image = cv::Mat(cv::Size(256, 256), CV_8UC3, cv::Scalar::all(0));
	}
	// ファイル名と解像度を保管しておく
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
** 投影テクスチャマッピングを行うようにする関数
** 投影テクスチャマッピング行うオブジェクトを描画する関数をこの関数のtrueとfalseで挟む
*/
inline void MyTexture::projectiveTextureMapping(bool toggle){
	if (toggle){
		// テクスチャ座標自動生成用の行列を作成
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

		// テクスチャと重なるよう平行移動
		glMatrixMode(GL_TEXTURE);
		glLoadIdentity();
		glTranslated(0.5, 0.5, 0.0);
		glScaled(1.0, 1.0, 1.0);

		// 頂点のオブジェクト空間における座標値を使ってマッピングする
		glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
		glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
		glTexGeni(GL_R, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
		glTexGeni(GL_Q, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);

		// テクスチャ座標生成関数の設定
		glTexGendv(GL_S, GL_OBJECT_PLANE, genfunc[0]);
		glTexGendv(GL_T, GL_OBJECT_PLANE, genfunc[1]);
		glTexGendv(GL_R, GL_OBJECT_PLANE, genfunc[2]);
		glTexGendv(GL_Q, GL_OBJECT_PLANE, genfunc[3]);

		// テクスチャマッピング開始
		glEnable(GL_TEXTURE_2D);
		// テクスチャ座標の自動生成を有効にする
		glEnable(GL_TEXTURE_GEN_S);
		glEnable(GL_TEXTURE_GEN_T);
		glEnable(GL_TEXTURE_GEN_R);
		glEnable(GL_TEXTURE_GEN_Q);
	}
	else{
		// テクスチャ座標の自動生成を無効にする
		glDisable(GL_TEXTURE_GEN_S);
		glDisable(GL_TEXTURE_GEN_T);
		glDisable(GL_TEXTURE_GEN_R);
		glDisable(GL_TEXTURE_GEN_Q);
		// テクスチャマッピング終了
		glDisable(GL_TEXTURE_2D);
	}
}
//inline void MyTexture::projectiveTextureMapping(bool toggle){
//	if (toggle){
//		// テクスチャ座標自動生成用の行列を作成
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
//		// テクスチャと重なるよう平行移動
//		glMatrixMode(GL_TEXTURE);
//		glLoadIdentity();
//		glTranslated(0.5, 0.5, 0.0);
//		glScaled(1.0, 1.0, 1.0);
//
//		// 頂点のオブジェクト空間における座標値を使ってマッピングする
//		glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
//		glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
//		glTexGeni(GL_R, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
//		glTexGeni(GL_Q, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
//
//		// テクスチャ座標生成関数の設定
//		glTexGendv(GL_S, GL_OBJECT_PLANE, genfunc[0]);
//		glTexGendv(GL_T, GL_OBJECT_PLANE, genfunc[1]);
//		glTexGendv(GL_R, GL_OBJECT_PLANE, genfunc[2]);
//		glTexGendv(GL_Q, GL_OBJECT_PLANE, genfunc[3]);
//
//		// テクスチャマッピング開始
//		glEnable(GL_TEXTURE_2D);
//		// テクスチャ座標の自動生成を有効にする
//		glEnable(GL_TEXTURE_GEN_S);
//		glEnable(GL_TEXTURE_GEN_T);
//		glEnable(GL_TEXTURE_GEN_R);
//		glEnable(GL_TEXTURE_GEN_Q);
//	}
//	else{
//		// テクスチャ座標の自動生成を無効にする
//		glDisable(GL_TEXTURE_GEN_S);
//		glDisable(GL_TEXTURE_GEN_T);
//		glDisable(GL_TEXTURE_GEN_R);
//		glDisable(GL_TEXTURE_GEN_Q);
//		// テクスチャマッピング終了
//		glDisable(GL_TEXTURE_2D);
//	}
//}


/*
** テクスチャをMat化する関数
*/
inline void MyTexture::texture2mat(GLenum MODE, int window_w, int window_h, cv::Mat &matdata)
{
	// テクスチャを画像化
	//TextureはGL_COLOR_ATTACHMENT0_EXTに割り当てられているとする
	glReadBuffer(MODE);// FBO : GL_COLOR_ATTACHMENT0_EXT, GL_FRONT, GL_BACK
	glReadPixels(0, 0, window_w, window_h, GL_BGRA, GL_UNSIGNED_BYTE, (void*)matdata.data);
	//cv::flip(matdata, matdata, 0);

}

/*
** 計測用の画像を投影するときに使用するレンダリング関数
** 関数呼び出し前に変換行列を初期値にセットしておくこと
*/
inline void MyTexture::textureRenderer(GLuint &texture)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	//バッファクリア

	// 陰影付けと隠面消去処理は行わない
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);

	// テクスチャマッピングを有効にする
	glBindTexture(GL_TEXTURE_2D, texture);
	glEnable(GL_TEXTURE_2D);

	// 正方形を描く
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

	// テクスチャマッピングを無効にする
	glDisable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, 0);

	glutSwapBuffers();

}

#endif