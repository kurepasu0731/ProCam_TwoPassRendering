#include "mygl.h"
#include "Header.h"


/*
** コード投影
*/
void myGL::getPixelCorrespondance(bool projection_image)
{
	//　グレイコードの投影＆撮影＆対応点取得
	if (projection_image){
		codeProjection();
	}
	gc.make_thresh();
	gc.makeCorrespondence();
	getWorldPoint();

	// ユーザ視点位置に置かれたカメラの透視投影行列の計算(3次元復元した後じゃないと計算できないので注意)
	cv::Mat perspectiveMat = calcUserPerspectiveMatrix(userTransVec);
	// ユーザ視点のGL変換行列の計算
	calcGLMatrix(devmatrix->user, perspectiveMat, CamWidth, CamHeight);
}


/*
** 構造化パターンを投影する関数
*/
void myGL::codeProjection()
{
	// GLの変換行列の初期化
	setRenderTargetWindow(window[PRJ].id);
	setIdentityMatrix(window[PRJ].width, window[PRJ].height);

	// 定数
	typedef enum flag{
		POSI = true,
		NEGA = false,
		VERTICAL = true,
		HORIZONTAL = false,
	} flag;

	//初期設定
	if (pgrOpenCV->init(FlyCapture2::PIXEL_FORMAT_BGR, FlyCapture2::HQ_LINEAR) == -1){
		exit(0);
	}
	pgrOpenCV->setShutterSpeed(pgrOpenCV->getShutter_PS());

	/* --- Gray Code の読み込み --- */
	// 投影枚数
	int all_bit = gc.c->g.all_bit;
	int h_bit = gc.c->g.h_bit;

	// 投影画像(ポジ&ネガ)格納用
	cv::Mat *posi_img = new cv::Mat[all_bit];  // ポジパターン用
	cv::Mat *nega_img = new cv::Mat[all_bit];  // ネガパターン用

	// 描画テクスチャ用
	GLuint *posi_texture = new GLuint[all_bit];  // ポジパターン用
	GLuint *nega_texture = new GLuint[all_bit];  // ネガパターン用

	// 書式付入出力（グレイコード読み込み用）
	std::stringstream *Filename_posi = new std::stringstream[all_bit];
	std::stringstream *Filename_nega = new std::stringstream[all_bit];
	// 書式付入出力（撮影画像書き込み用）
	std::stringstream *Filename_posi_cam = new std::stringstream[all_bit];
	std::stringstream *Filename_nega_cam = new std::stringstream[all_bit];

	std::cout << "投影用グレイコード画像読み込み中" << std::endl;
	for (unsigned int i = 0; i < all_bit; i++) {
		Filename_posi[i] << "./GrayCodeImage/ProjectionGrayCode/posi" << std::setw(2) << std::setfill('0') << i << ".bmp";
		Filename_nega[i] << "./GrayCodeImage/ProjectionGrayCode/nega" << std::setw(2) << std::setfill('0') << i << ".bmp";
		// 読み込み
		posi_img[i] = cv::imread(Filename_posi[i].str(), 1);
		nega_img[i] = cv::imread(Filename_nega[i].str(), 1);
		Filename_posi[i] << std::endl;
		Filename_nega[i] << std::endl;

		// 読み込む枚数が足りなかったらグレイコード画像を作り直す
		if (posi_img[i].empty() || nega_img[i].empty()){
			std::cout << "ERROR(1)：投影用のグレイコード画像が不足しています。" << std::endl;
			std::cout << "ERROR(2)：グレイコード画像を作成します。" << std::endl;
			gc.makeGraycodeImage();
			codeProjection();
			return;
		}
		setTexture(posi_img[i], posi_texture[i]);
		setTexture(nega_img[i], nega_texture[i]);
	}

	// 白色画像の作成
	cv::Mat white = cv::Mat(ProjHeight, ProjWidth, CV_8UC3, cv::Scalar(255, 255, 255));
	GLuint w_texture;
	setTexture(white, w_texture);

	std::cout << "投影開始" << std::endl;

	// posi画像の投影撮影
	for (unsigned int i = 0; i < all_bit; i++) {
		cv::Mat cap;
		// レンダリング
		textureRenderer(posi_texture[i]);
		Sleep(delay * 1.5);
		// 撮影画像をmatに格納
		pgrOpenCV->CameraCapture(cap);	//13ms
		// 横縞
		if (i < h_bit)
			Filename_posi_cam[i] << "./GrayCodeImage/CaptureImage/CameraImg" << HORIZONTAL << "_" << std::setw(2) << std::setfill('0') << i + 1 << "_" << POSI << ".bmp";
		// 縦縞
		else
			Filename_posi_cam[i] << "./GrayCodeImage/CaptureImage/CameraImg" << VERTICAL << "_" << std::setw(2) << std::setfill('0') << i - h_bit + 1 << "_" << POSI << ".bmp";

		// 保存
		cv::imwrite(Filename_posi_cam[i].str(), cap);
		Filename_posi_cam[i] << std::endl;
	}

	// nega画像の投影撮影
	for (unsigned int i = 0; i < all_bit; i++) {
		cv::Mat cap;
		// レンダリング
		textureRenderer(nega_texture[i]);
		Sleep(delay * 1.5);
		// 撮影画像をmatに格納
		pgrOpenCV->CameraCapture(cap);	//13ms
		if (i < h_bit)
			Filename_nega_cam[i] << "./GrayCodeImage/CaptureImage/CameraImg" << HORIZONTAL << "_" << std::setw(2) << std::setfill('0') << i + 1 << "_" << NEGA << ".bmp";
		// 縦縞
		else
			Filename_nega_cam[i] << "./GrayCodeImage/CaptureImage/CameraImg" << VERTICAL << "_" << std::setw(2) << std::setfill('0') << i - h_bit + 1 << "_" << NEGA << ".bmp";
		// 保存
		cv::imwrite(Filename_nega_cam[i].str(), cap);
		Filename_nega_cam[i] << std::endl;
	}

	cv::Mat cap;
	// 白色画像をレンダリング
	textureRenderer(w_texture);
	Sleep(delay * 1.5);
	pgrOpenCV->CameraCapture(cap);	//13ms
	cv::imwrite(PROJECTION_SURFACE_ADDRESS, cap);

	glBindTexture(GL_TEXTURE_2D, 0);

	/***** 終了 *****/
	// テクスチャの消去
	glDeleteTextures(all_bit * 2, posi_texture);
	glDeleteTextures(all_bit * 2, nega_texture);
	glDeleteTextures(1 * 2, &w_texture);
	// 解放
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
** 投影面の3次元座標を格納する関数
*/
void myGL::getWorldPoint()
{
	//プロジェクタ解像度分データを取得
	//エラー画素には -1 を格納
	gc.getCorrespondAllPoints(projPoint, imagePoint, pointColor);
	// 対応点の歪み除去
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

	// 3次元座標の取得
	calib.reconstruction(reconstructPoint, undistort_projPoint, undistort_imagePoint);

	//平滑化処理
	smoothing();
}


/*
** 投影面の平滑化処理
*/
void myGL::smoothing()
{

}

/*
** user view の透視投影行列を計算する関数
** 3次元再構成を行った後に呼ばないといけない
** ユーザの位置情報を取得後に呼ばないといけない
*/
cv::Mat myGL::calcUserPerspectiveMatrix(cv::Point3f userPos)
{
	// 3次元復元されていない場合適当な値を返す
	if (reconstructPoint.size() == 0){
		std::cout << "user_trans is null point" << std::endl;
		cv::Mat n;
		return n;
	}

	// ユーザの位置へカメラを移動する
	// 回転行列(PGRカメラからユーザへの回転。実際使用するのはユーザからPGRカメラへの回転)
	cv::Mat user_R(3, 1, CV_64F);
	user_R.at<double>(0, 0) = 0;	// roll(x軸回りの回転)
	user_R.at<double>(1, 0) = 0;	// pich(y軸回りの回転)
	user_R.at<double>(2, 0) = 0;	// yaw(z軸回りの回転)
	cv::Rodrigues(user_R, user_R);
	cv::Mat user_R_inv = user_R.t();

	// 並進行列(userPosはPGRカメラ座標系におけるユーザの位置ベクトル。実際使用するのはユーザ座標系におけるPGRカメラの位置ベクトル
	// つまり座標変換が必要
	cv::Mat user_T(3, 1, CV_64F);
	user_T.at<double>(0, 0) = userPos.x;	// roll(x軸回りの回転)
	user_T.at<double>(1, 0) = userPos.y;	// pich(y軸回りの回転)
	user_T.at<double>(2, 0) = userPos.z;	// yaw(z軸回りの回転)
	cv::Mat worldPoint(3, 1, CV_64F);
	worldPoint.at<double>(0, 0) = 0.0;
	worldPoint.at<double>(1, 0) = 0.0;
	worldPoint.at<double>(2, 0) = 0.0;
	cv::Mat Tvec = -1.0 * user_R_inv * user_T;

	// 回転と並進を結合
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

	// 内部パラメータの変形（観測カメラの内部パラメータを使用）
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
