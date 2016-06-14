#include "mygl.h"

myGL::myGL()
{

	// キャリブレーションデータの読み込み
	calib.loadCalibParam(PROCAM_CALIBRATION_RESULT_FILENAME);

	// パラメータ初期化
	devmatrix = new DeviceMatrix();
	pgrOpenCV = new TPGROpenCV(0);
	imagePoint.clear();
	projPoint.clear();
	pointColor.clear();
	reconstructPoint.clear();

	// パラメータの読み込み
	char buf[128];
	GetPrivateProfileStringA("delay", "delay", NULL, buf, sizeof(buf), "./parameter.ini");
	delay = double(atoi(buf));

	// 各種ウインドウの設定
	initWindow();

	// カメラ側の変換行列の計算
	cv::Mat perspectiveMatrix = calib.getCamPerspectiveMat();
	calcGLMatrix(devmatrix->cam, perspectiveMatrix, CamWidth, CamHeight);

	// プロジェクタ側の変換行列の計算
	perspectiveMatrix = calib.getProjPerspectiveMat();
	calcGLMatrix(devmatrix->proj, perspectiveMatrix, ProjWidth, ProjHeight);


	// フラグ設定
	exeGeomtric = false;

	// 他視点用のキャリブレーションデータの読み込み
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
** window情報の設定を行う
*/
void myGL::initWindow()
{
	// Window 0 の設定
	window[PRJ].flag = 1;
	window[PRJ].positionX = 1681;
	window[PRJ].positionY = 0;
	window[PRJ].width = ProjWidth;
	window[PRJ].height = window[PRJ].width * ((double)ProjHeight / (double)ProjWidth);
	window[PRJ].title = "Projector View";

	// Window 1 の設定
	window[CAM].flag = 0;
	window[CAM].positionX = 0;
	window[CAM].positionY = 600;
	window[CAM].width = 400;
	window[CAM].height = window[CAM].width * ((double)CamHeight / (double)CamWidth);
	window[CAM].title = "Camera View";

	// Window 2 の設定
	window[USER].flag = 0;
	window[USER].positionX = 800;
	window[USER].positionY = 600;
	window[USER].width = 400;
	window[USER].height = window[USER].width * ((double)CamHeight / (double)CamWidth);
	window[USER].title = "User View";
}

/*
** windowの初期設定を行う関数
*/
void myGL::createWindow(int window_num)
{
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);	//表示モード
	glutInitWindowPosition(window[window_num].positionX, window[window_num].positionY); //ウィンドウの位置の指定
	glutInitWindowSize(window[window_num].width, window[window_num].height); //ウィンドウサイズの指定
	window[window_num].id = glutCreateWindow(window[window_num].title); //ウィンドウの名前
}

/*
** 最初に一度だけ呼ばれる関数
*/
void myGL::initialize()
{
	// カメラをスタートさせる
	if (pgrOpenCV->init(FlyCapture2::PIXEL_FORMAT_BGR, FlyCapture2::HQ_LINEAR) != -1) {	// カメラが正常に動作していた場合
		pgrOpenCV->setShutterSpeed(pgrOpenCV->getShutter_h());
		pgrOpenCV->start();	// カメラスタート
	}
	else {
		std::cout << "camera connection error" << std::endl;
		exit(-1);
	}

	// カメラ、プロジェクタの歪みマップの算出
	proj_dist_inv = calib.proj_dist * -1;	// プロジェクタ側のレンダリング結果を歪ませる行列
	cv::initUndistortRectifyMap(calib.cam_K, calib.cam_dist, cv::Mat(), calib.cam_K, cv::Size(CamWidth, CamHeight), CV_32FC1, camUndistMap1, camUndistMap2);
	cv::initUndistortRectifyMap(calib.proj_K, proj_dist_inv, cv::Mat(), calib.proj_K, cv::Size(ProjWidth, ProjHeight), CV_32FC1, projDistMap1, projDistMap2);

	// 画像の読み込み
	targetIndex = 0;
	if (loadInputImages(IMAGE_DIRECTORY, originalImages) == -1){
		std::cout << "投影用の画像データがありません" << std::endl;
		exit(-1);
	}
	targetImage = originalImages[targetIndex];

	// 他視点対応用の変数
	userTransVec = cv::Point3f(0, 0, 0);	// ユーザの初期位置ベクトル（単位[mm]）
	userPos = cv::Mat(4, 1, CV_64F);		// ユーザの位置ベクトルを格納するmat (同次座標表現)

	std::cout << "r：リセット" << std::endl;
	std::cout << "q：終了" << std::endl;
	std::cout << "0：コード投影あり" << std::endl;
	std::cout << "1：コード投影なし" << std::endl;
	std::cout << "→ : 画像変更" << std::endl;
	std::cout << "← : 画像変更" << std::endl;
}
/*
** openGLの変換行列を計算する関数
*/
void myGL::calcGLMatrix(GLMatrix &myglmatrix, cv::Mat &perspectiveMatrix, int w, int h)
{
	// カメラ視点の変換行列の計算
	double CONVERT_TO_OPENGL_WIN[16] = {
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1
	};

	// ？クリッピング領域の正規化+座標変換
	CONVERT_TO_OPENGL_WIN[0] = 1.0 / (w / 2.0);
	CONVERT_TO_OPENGL_WIN[5] = -1.0 / (h / 2.0);
	CONVERT_TO_OPENGL_WIN[12] = -1.0;
	CONVERT_TO_OPENGL_WIN[13] = 1.0;

	// 射影変換行列の設定
	double projection[16];
	for (int i = 0; i < 4; i++){
		projection[i * 4] = perspectiveMatrix.at<double>(0, i);
		projection[i * 4 + 1] = perspectiveMatrix.at<double>(1, i);
		projection[i * 4 + 2] = 0;
		projection[i * 4 + 3] = perspectiveMatrix.at<double>(2, i);
	}

	// OpenGLのGL_PROJECTION モードのときに使用する射影変換行列を計算
	cv::Mat OpenGLWin(4, 4, CV_64FC1, CONVERT_TO_OPENGL_WIN);
	cv::Mat ProjctionMat(4, 4, CV_64FC1, projection);
	ProjctionMat = ProjctionMat * OpenGLWin; // 行列を転置するとOpenGLの配列の順番と一致
	for (int i = 0; i < 4; i++){
		for (int j = 0; j < 4; j++){
			myglmatrix.projectionMatrix[i * 4 + j] = ProjctionMat.at<double>(i, j); // ラスタースキャン
		}
	}

	// テクスチャマトリックスを計算
	static double modelview[16] = {
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1
	};
	// テクスチャ座標系にあわせる行列
	double convert[16] = {
		0.5, 0, 0, 0.5,
		0, -0.5, 0, 0.5,
		0, 0, 1, 0,
		0, 0, 0, 1
	};
	cv::Mat Model(4, 4, CV_64FC1, modelview);
	cv::Mat Texture(4, 4, CV_64FC1, convert);
	cv::Mat PM = Model * ProjctionMat;
	cv::transpose(PM, PM); //転置
	PM = Texture * PM;
	int index = 0;
	for (int i = 0; i < 4; i++){
		for (int t = 0; t < 4; t++){
			myglmatrix.textureMatrix[i][t] = cv::saturate_cast<double>(PM.at<double>(i, t));
		}
	}

	// 縦横解像度を保持
	myglmatrix.width = w;
	myglmatrix.height = h;
}

/*
** WindowごとにOpenGLの設定を行う関数
*/
void myGL::initGLsetting(int window_num)
{

	/* Window1 (プロジェクタ視点映像) の設定 */
	if (window_num == PRJ)
	{
		glClearColor(0.0, 0.0, 0.0, 1.0);
		glEnable(GL_DEPTH_TEST);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		// GLEW の初期化
		GLenum err = glewInit();
		if (err != GLEW_OK) {
			fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
			exit(1);
		}
	}

	/* Window0 (カメラ視点映像) の設定 */
	if (window_num == CAM)
	{
		glClearColor(0.0, 0.0, 0.0, 1.0);
		glEnable(GL_DEPTH_TEST);
		// OpenGLの射影行列を設定する
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
	}

	/* Window2 (ユーザ視点映像) の設定 */
	if (window_num == USER)
	{
		glClearColor(0.0, 0.0, 0.0, 1.0);
		glEnable(GL_DEPTH_TEST);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
	}
}

//************************
//		描画
//************************

/* プロジェクタ側の描画処理 */
void myGL::display_projector_view()
{
	if (exeGeomtric == true){

		// twopass rendering により幾何補正画像を作成
		cv::remap(targetImage, undistCamImage, camUndistMap1, camUndistMap2, cv::INTER_LINEAR);	// 歪みの除去
		convertImageCoordinateUsingTwoPassRendering(undistCamImage, undistProjImage, devmatrix->cam, devmatrix->proj); //プロジェクタ解像度へ
		cv::remap(undistProjImage, distortProjImg, projDistMap1, projDistMap2, cv::INTER_LINEAR);	// 歪みを与える
		// テクスチャ化
		setTexture(distortProjImg, projectionTexture);

		// 各変換行列を単位行列にする
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glViewport(0, 0, window[PRJ].width, window[PRJ].height);	// ビューポートはウィンドウのサイズに合わせる

		// 補正画像の投影
		textureRenderer(projectionTexture);
		glDeleteTextures(1, &projectionTexture);	// 使い終わったテクスチャを削除

		// PGR撮影画像の表示
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

/* カメラ側の描画処理 */
void myGL::display_camera_view()
{
	if (exeGeomtric == true){

		setTexture(targetImage, cam_texture);	// テクスチャ化
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glMultMatrixd(devmatrix->cam.projectionMatrix);

		/* テクスチャ行列の設定 */
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		// 射影マッピングの位置をユーザ視点に設定
		glMatrixMode(GL_TEXTURE);

		// 自動生成の計算式にオブジェクト空間の頂点座標を使う。
		glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
		glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
		glTexGeni(GL_R, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
		glTexGeni(GL_Q, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);

		// 合成した変換行列をオブジェクトの頂点に掛ければ画面を覆うようにUVが計算される。
		glTexGenfv(GL_S, GL_OBJECT_PLANE, devmatrix->cam.textureMatrix[0]);
		glTexGenfv(GL_T, GL_OBJECT_PLANE, devmatrix->cam.textureMatrix[1]);
		glTexGenfv(GL_R, GL_OBJECT_PLANE, devmatrix->cam.textureMatrix[2]);
		glTexGenfv(GL_Q, GL_OBJECT_PLANE, devmatrix->cam.textureMatrix[3]);

		// UVの自動生成を有効化する。
		glEnable(GL_TEXTURE_GEN_S);
		glEnable(GL_TEXTURE_GEN_T);
		glEnable(GL_TEXTURE_GEN_R);  // おまけ
		glEnable(GL_TEXTURE_GEN_Q);

		/* 画面クリア */
		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		//テクスチャマッピング開始
		glBindTexture(GL_TEXTURE_2D, cam_texture);
		glEnable(GL_TEXTURE_2D);
		pointCloudRender();
		//テクスチャマッピング終了
		glDisable(GL_TEXTURE_2D);

		// UVの自動生成を有効化する。
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
** ユーザ視点のレンダリング
*/
void myGL::display_user_view()
{
	if (exeGeomtric == true){

		setTexture(targetImage, user_texture);	// テクスチャ化
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glMultMatrixd(devmatrix->user.projectionMatrix);

		/* テクスチャ行列の設定 */
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		// 射影マッピングの位置をユーザ視点に設定
		glMatrixMode(GL_TEXTURE);

		// 自動生成の計算式にオブジェクト空間の頂点座標を使う。
		glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
		glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
		glTexGeni(GL_R, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
		glTexGeni(GL_Q, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);

		// 合成した変換行列をオブジェクトの頂点に掛ければ画面を覆うようにUVが計算される。
		glTexGenfv(GL_S, GL_OBJECT_PLANE, devmatrix->cam.textureMatrix[0]);
		glTexGenfv(GL_T, GL_OBJECT_PLANE, devmatrix->cam.textureMatrix[1]);
		glTexGenfv(GL_R, GL_OBJECT_PLANE, devmatrix->cam.textureMatrix[2]);
		glTexGenfv(GL_Q, GL_OBJECT_PLANE, devmatrix->cam.textureMatrix[3]);

		// UVの自動生成を有効化する。
		glEnable(GL_TEXTURE_GEN_S);
		glEnable(GL_TEXTURE_GEN_T);
		glEnable(GL_TEXTURE_GEN_R);  // おまけ
		glEnable(GL_TEXTURE_GEN_Q);

		/* 画面クリア */
		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		//テクスチャマッピング開始
		glBindTexture(GL_TEXTURE_2D, user_texture);
		glEnable(GL_TEXTURE_2D);
		pointCloudRender();
		//テクスチャマッピング終了
		glDisable(GL_TEXTURE_2D);

		// UVの自動生成を有効化する。
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

/* アイドル時の処理 */
void myGL::idle()
{
	for (int loop = 0; loop < WindowNum; ++loop){
		if (window[loop].flag > 0) {
			glutSetWindow(window[loop].id);
			glutPostRedisplay(); //再描画 (※display()関数を呼び出す関数 )
		}
	}
}

/* マウスクリック時の処理 */
void myGL::mouseClick(int button, int state, int x, int y)
{

}

/* マウスドラッグ時の処理 */
void myGL::mouseMotion(int x, int y)
{

}

/* マウスホイール操作時の処理 */
void myGL::mouseWheel(int wheel_number, int direction, int x, int y)
{
}

/* 一定間隔で呼ばれる関数 */
void myGL::timer(int value)
{
}

/* glutのループ終了時に呼ばれる関数 */
void myGL::close()
{
}

//視点変更
void myGL::polarview(){

}

/* リサイズ時の処理 */
void myGL::reshape_camera_view(int w, int h)
{
	window[CAM].width = w;
	window[CAM].height = h;
	glViewport(0, 0, w, h);	//描画サイズ固定化
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMultMatrixd(devmatrix->cam.projectionMatrix);
	glutPostRedisplay();	//glutDisplayFunc()を１回実行する
}
void myGL::reshape_projector_view(int w, int h)
{
	window[PRJ].width = w;
	window[PRJ].height = h;
	glViewport(0, 0, w, h);	//描画サイズ固定化
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMultMatrixd(devmatrix->proj.projectionMatrix);
	glutPostRedisplay();	//glutDisplayFunc()を１回実行する
}

void myGL::reshape_user_view(int w, int h)
{
	window[USER].width = w;
	window[USER].height = h;
	glViewport(0, 0, w, h);	//描画サイズ固定化
	glutPostRedisplay();	//glutDisplayFunc()を１回実行する
}

/* キーボード操作時の処理 */
void myGL::keyboard(unsigned char key, int x, int y)
{
	switch (key){
	case 'q':	//終了
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

	std::cout << "*******入力待ち*********" << std::endl;
}
/*
** 特殊キー入力受付用
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
	std::cout << "*******入力待ち*********" << std::endl;
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
	// ユーザ視点位置に置かれたカメラの透視投影行列の計算(3次元復元した後じゃないと計算できないので注意)
	cv::Mat perspectiveMat = calcUserPerspectiveMatrix(userTransVec);
	// ユーザ視点のGL変換行列の計算
	calcGLMatrix(devmatrix->user, perspectiveMat, CamWidth, CamHeight);

	std::cout << "*******入力待ち*********" << std::endl;
}

//*****************************************
//				パーツ群
//*****************************************

/*
** レンダリングするウインドウを指定する関数
*/
void myGL::setRenderTargetWindow(int window_id)
{
	// 描画するウインドウをセット
	glutSetWindow(window_id);
}

/*
** OpneGLの変換行列をリセットする関数
*/
void myGL::setIdentityMatrix(int viewport_w, int viewpoert_h)
{
	// GLの変換行列の初期化
	glViewport(0, 0, viewport_w, viewpoert_h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMatrixMode(GL_TEXTURE);
	glLoadIdentity();
}

/*
** 投影面を点群として描画する
*/
void myGL::pointCloudRender()
{
	glBegin(GL_POINTS);
	glPointSize(1);
	for (int i = 0; i < reconstructPoint.size(); ++i) {
		//対応が取れていれば描画
		if (projPoint[i].x != -1){
			glColor3f(pointColor[i].x / 255.0f, pointColor[i].y / 255.0f, pointColor[i].z / 255.0f); //RGB
			glVertex3f(reconstructPoint[i].x, reconstructPoint[i].y, reconstructPoint[i].z);
		}
	}
	glEnd();
}


/*
** テクスチャを作成する
** 外部から呼び出し用
*/
void myGL::setTexture(cv::Mat &src, GLuint &texture)
{
	myTex.setTexture(src, &texture);
}
/*
** 計測用の画像を投影するときに使用するレンダリング関数
** 関数呼び出し前に変換行列を初期値にセットしておくこと
*/
void myGL::textureRenderer(GLuint &texture)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	//バッファクリア

	// 陰影付けと隠面消去処理は行わない
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);
	//glEnable(GL_DEPTH_TEST);

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
	glDisable(GL_DEPTH_TEST);

	glutSwapBuffers();

}


/**
@brief 下位ディレクトリ内の全ファイルの名前を取得（一括変換用）
*/
std::vector<std::string> myGL::getAllFileName(std::string searchDir)
{
	std::vector<std::string> file_list;

	// カレントディレクトリ以下のファイル名を取得する
	// 再帰的にファイル名を取得する場合は、std::tr2::sys::recursive_directory_iteratorを使う
	for (std::tr2::sys::directory_iterator it(searchDir), end; it != end; ++it) {
		// 画像ファイルだけ取得
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

	// 取得したファイル名をすべて表示する
	//for (auto &path : file_list) {
	//	std::cout << path << std::endl;
	//}
	return file_list;
}


/*
** 投影する画像を読み込む関数
*/
int myGL::loadInputImages(std::string searchDir, std::vector<cv::Mat> &images)
{
	// ファイルパスの取得
	fileName = getAllFileName(searchDir);
	// 画像の読み込み
	fileNum = fileName.size();
	// 読み込む画像が見つからなかった場合
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
** テクスチャを保存する関数
*/
void myGL::saveImage(const unsigned int imageWidth, const unsigned int imageHeight)
{
	const unsigned int channelNum = 3; // RGBなら3, RGBAなら4
	void* dataBuffer = NULL;
	dataBuffer = (GLubyte*)malloc(imageWidth * imageHeight * channelNum);

	// 読み取るOpneGLのバッファを指定 GL_FRONT:フロントバッファ　GL_BACK:バックバッファ
	glReadBuffer(GL_FRONT);

	// OpenGLで画面に描画されている内容をバッファに格納
	glReadPixels(
		0,                 //読み取る領域の左下隅のx座標
		0,                 //読み取る領域の左下隅のy座標 //0 or getCurrentWidth() - 1
		imageWidth,             //読み取る領域の幅
		imageHeight,            //読み取る領域の高さ
		GL_BGR, //it means GL_BGR,           //取得したい色情報の形式
		GL_UNSIGNED_BYTE,  //読み取ったデータを保存する配列の型
		dataBuffer      //ビットマップのピクセルデータ（実際にはバイト配列）へのポインタ
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
@brief 2パスレンダリングをオフスクリーンで行う関数
@details
@param src 射影マッピング用画像
@param dst レンダリング結果
@param projector 1pass目のデバイス
@param camera 2pass目のデバイス
*/
void myGL::convertImageCoordinateUsingTwoPassRendering(cv::Mat &src, cv::Mat &dst, GLMatrix &projector, GLMatrix &camera)
{
	// オフスクリーンレンダリングする際のレンダリング結果の解像度を指定
	int dstWidth = camera.width;
	int dstHeight = camera.height;

	// GLの変換行列の初期化
	setRenderTargetWindow(window[PRJ].id);
	setIdentityMatrix(dstWidth, dstHeight);

	// フレームバッファオブジェクト用変数
	GLuint fboID, _cb, _rb;
	myTex.setFramebufferTexture(&fboID, &_cb, &_rb, dstWidth, dstHeight);
	GLuint Texture;
	setTexture(src, Texture);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMultMatrixd(camera.projectionMatrix);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// フレームバッファオブジェクトを結合する
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fboID);

	//テクスチャマッピング開始
	glBindTexture(GL_TEXTURE_2D, Texture);
	glEnable(GL_TEXTURE_2D);

	// UVの自動生成を有効化する。
	glEnable(GL_TEXTURE_GEN_S);
	glEnable(GL_TEXTURE_GEN_T);
	glEnable(GL_TEXTURE_GEN_R);  // おまけ
	glEnable(GL_TEXTURE_GEN_Q);

	// 自動生成の計算式にオブジェクト空間の頂点座標を使う。
	glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
	glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
	glTexGeni(GL_R, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
	glTexGeni(GL_Q, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);

	// 合成した変換行列をオブジェクトの頂点に掛ければ画面を覆うようにUVが計算される。
	glTexGenfv(GL_S, GL_OBJECT_PLANE, projector.textureMatrix[0]);
	glTexGenfv(GL_T, GL_OBJECT_PLANE, projector.textureMatrix[1]);
	glTexGenfv(GL_R, GL_OBJECT_PLANE, projector.textureMatrix[2]);
	glTexGenfv(GL_Q, GL_OBJECT_PLANE, projector.textureMatrix[3]);

	/* モデルビュー変換行列の設定 */
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	///* 画面クリア */
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// 3次元形状を復元
	pointCloudRender();
	// テクスチャ座標の自動生成を無効にする
	glDisable(GL_TEXTURE_GEN_S);
	glDisable(GL_TEXTURE_GEN_T);
	glDisable(GL_TEXTURE_GEN_R);
	glDisable(GL_TEXTURE_GEN_Q);
	//テクスチャマッピング終了
	glDisable(GL_TEXTURE_2D);

	///* テクスチャマッピングを無効にする*/
	glDisable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, 0);

	// 描画実行
	glFlush();

	// テクスチャを画像化
	dst = cv::Mat(cv::Size(dstWidth, dstHeight), CV_8UC3);	//RGBAでなくてもよい
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);// FBO : GL_COLOR_ATTACHMENT0_EXT, GL_FRONT, GL_BACK
	glReadPixels(0, 0, dstWidth, dstHeight, GL_BGR, GL_UNSIGNED_BYTE, (void*)dst.data);
	cv::flip(dst, dst, 0);

	// フレームバッファオブジェクトの結合を解除する(画像に読み込んだ後に，バインド解除)
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
	myTex.deleteFramebufferTexture(&fboID, &_cb, &_rb);
	glDeleteTextures(1 * 2, &Texture);

	setIdentityMatrix(window[PRJ].width, window[PRJ].height);
}
