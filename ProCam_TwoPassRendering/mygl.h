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
		int positionX;	// ウィンドウ位置のX座標
		int positionY;	// ウィンドウ位置のY座標
		int width;		// ウィンドウの幅
		int height;		// ウィンドウの高さ
		char* title;		// ウィンドウのタイトル
		int flag;			//ウィンドウのon off
	} Windows;

	typedef struct _GLMatrix {
		double projectionMatrix[16];
		float textureMatrix[4][4];
		int width;
		int height;
	} GLMatrix;

	// OpenGLの変換行列をまとめるための型
	typedef struct _DeviceMatrix {
		GLMatrix cam;
		GLMatrix proj;
		GLMatrix user;
	} DeviceMatrix;

	typedef enum opengl_window{
		PRJ, CAM, USER
	};

	// OpenGLの変換行列をまとめてある変数
	DeviceMatrix *devmatrix;

	static const int WindowNum = 3;
	Windows window[WindowNum];

	//カメラとプロジェクタの情報
	const static int CamWidth = CAMERA_WIDTH;
	const static int CamHeight = CAMERA_HEIGHT;
	const static int ProjWidth = PROJECTOR_WIDTH;
	const static int ProjHeight = PROJECTOR_HEIGHT;
	
	// フラグ
	bool exeGeomtric;

	// コールバック関数で呼ばせる関数
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
	void special_key(int key, int x, int y);		// window0 操作用
	void special_key_user(int key, int x, int y);	//window 3 操作用
	void close();
	void timer(int value);
	void polarview();

	/**************/
	/* メンバ関数 */
	/**************/

	// 初期化で使う関数
	void initWindow();					// windowの位置やサイズを指定する関数
	void createWindow(int window_num);	// windowの初期設定を行う関数
	void initGLsetting(int window_num);	// glの初期設定
	void initialize();					// プログラムの初期設定を行う関数
	void calcGLMatrix(GLMatrix &myglmatrix, cv::Mat &perspectiveMatrix, int w, int h);	// OpneGLの変換行列を計算する関数

	// その他
	void setRenderTargetWindow(int window_id);
	void setIdentityMatrix(int viewport_w, int viewpoert_h);					
	void pointCloudRender();													// 3次元点群描画関数
	void setTexture(cv::Mat &src, GLuint &texture);								// Mat⇒テクスチャにする関数
	void textureRenderer(GLuint &texture);										// テクスチャをレンダリングする関数
	std::vector<std::string> getAllFileName(std::string serchDir);				// 投影用画像ファイルの読み込みに使用する関数
	int loadInputImages(std::string searchDir, std::vector<cv::Mat> &images);	// 投影用画像を一括読み込みする関数
	void saveImage(const unsigned int imageWidth, const unsigned int imageHeight);

	//　計測用
	void getPixelCorrespondance(bool projection_image);		//コード投影から3次元点群取得まで行う関数
	void getWorldPoint();		// 3次元座標を取得する関数
	void codeProjection();		// グレイコード投影を行う関数
	void smoothing();			// 平滑化を行う関数
	void convertImageCoordinateUsingTwoPassRendering(cv::Mat &src, cv::Mat &dst, GLMatrix &projector, GLMatrix &camera); // オフスクリーンレンダリングを行う関数

private:
	// 遅延量
	float delay;

	// 対応点の取得
	std::vector<cv::Point2f> imagePoint;	// カメラ座標
	std::vector<cv::Point2f> projPoint;		// プロジェクタ座標
	std::vector<cv::Point3i> pointColor;	// カラー情報
	std::vector<cv::Point3f> reconstructPoint;	// 3次元座標

	// レンダリング用テクスチャ
	GLuint projectionTexture;
	GLuint cam_texture;
	GLuint user_texture;

	// 画像データ
	cv::Mat targetImage;		//目標画像
	cv::Mat capImage;			//撮影画像
	cv::Mat distortProjImg;		//歪ませたプロジェクタ画像
	cv::Mat undistCamImage;		//two pass用補正画像
	cv::Mat undistProjImage;	//two pass用補正画像

	// 歪み係数
	cv::Mat proj_dist_inv, projDistMap1, projDistMap2, camUndistMap1, camUndistMap2;

	// 画像ファイル一括取得用
	std::vector<cv::Mat> originalImages;	
	std::vector < std::string > fileName;
	int fileNum;
	int targetIndex;	// 現在投影中の画像番号

	// ユーザ位置への並進
	cv::Point3f userTransVec;
	cv::Mat userPos;
	cv::Mat calcUserPerspectiveMatrix(cv::Point3f user_T);	// ユーザ視点の透視投影行列を計算する関数
};



#endif
