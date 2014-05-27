// Calibrate.cpp : �R���\�[�� �A�v���P�[�V�����̃G���g�� �|�C���g���`���܂��B
//

#include "stdafx.h"
#include "opencv.h"

///���s���鑀���I��
///1:���s
///0:���s���Ȃ�
#define CALIB 1 //�J�����L�����u���[�V�����@
#define FIXIMG 0  //�c�ݕ␳(�摜)
#define FIX 0   //�c�ݕ␳(�|�C���g)

#define IMAGE_NUM  (30)         /* �摜�� */
//#define PAT_ROW    (4)          /* �p�^�[���̍s�� */
//#define PAT_COL    (5)         /* �p�^�[���̗� */
#define PAT_ROW    (7)          /* �p�^�[���̍s�� */
#define PAT_COL    (10)         /* �p�^�[���̗� */
#define PAT_SIZE   (PAT_ROW*PAT_COL)
#define ALL_POINTS (IMAGE_NUM*PAT_SIZE)
//#define CHESS_SIZE (40.0)       /* �p�^�[��1�}�X��1�ӃT�C�Y[mm] */
#define CHESS_SIZE (24.0)       /* �p�^�[��1�}�X��1�ӃT�C�Y[mm] */

int
	main (int argc, char *argv[])
{

#if CALIB
	int i, j, k;
	int corner_count, found;
	int p_count[IMAGE_NUM];
	IplImage *src_img[IMAGE_NUM];
	CvSize pattern_size = cvSize (PAT_COL, PAT_ROW);
	CvPoint3D32f objects[ALL_POINTS];
	CvPoint2D32f *corners = (CvPoint2D32f *) cvAlloc (sizeof (CvPoint2D32f) * ALL_POINTS);
	CvMat object_points;
	CvMat image_points;
	CvMat point_counts;
	CvMat *intrinsic = cvCreateMat (3, 3, CV_32FC1);
	CvMat *rotation = cvCreateMat (1, 3, CV_32FC1);
	CvMat *translation = cvCreateMat (1, 3, CV_32FC1);
	CvMat *distortion = cvCreateMat (1, 4, CV_32FC1);

	// (1)�L�����u���[�V�����摜�̓ǂݍ���
	for (i = 0; i < IMAGE_NUM; i++) {
		char buf[64];
		sprintf (buf, "data/images/img (%d).jpg", i+1);
		if ((src_img[i] = cvLoadImage (buf, CV_LOAD_IMAGE_COLOR)) == NULL) {
			fprintf (stderr, "cannot load image file : %s\n", buf);
		}
	}

	// (2)3������ԍ��W�̐ݒ�
	for (i = 0; i < IMAGE_NUM; i++) {
		for (j = 0; j < PAT_ROW; j++) {
			for (k = 0; k < PAT_COL; k++) {
				objects[i * PAT_SIZE + j * PAT_COL + k].x = j * CHESS_SIZE;
				objects[i * PAT_SIZE + j * PAT_COL + k].y = k * CHESS_SIZE;
				objects[i * PAT_SIZE + j * PAT_COL + k].z = 0.0;
			}
		}
	}
	cvInitMatHeader (&object_points, ALL_POINTS, 3, CV_32FC1, objects);

	// (3)�`�F�X�{�[�h�i�L�����u���[�V�����p�^�[���j�̃R�[�i�[���o
	int found_num = 0;
	//cvNamedWindow ("Calibration", CV_WINDOW_AUTOSIZE);
	for (i = 0; i < IMAGE_NUM; i++) {
		found = cvFindChessboardCorners (src_img[i], pattern_size, &corners[i * PAT_SIZE], &corner_count);
		fprintf (stderr, "%02d...", i);
		if (found) {
			fprintf (stderr, "ok\n");
			found_num++;
		}
		else {
			fprintf (stderr, "fail\n");
		}
		// (4)�R�[�i�[�ʒu���T�u�s�N�Z�����x�ɏC���C�`��
		IplImage *src_gray = cvCreateImage (cvGetSize (src_img[i]), IPL_DEPTH_8U, 1);
		cvCvtColor (src_img[i], src_gray, CV_BGR2GRAY);
		cvFindCornerSubPix (src_gray, &corners[i * PAT_SIZE], corner_count,
			cvSize (3, 3), cvSize (-1, -1), cvTermCriteria (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));
		cvDrawChessboardCorners (src_img[i], pattern_size, &corners[i * PAT_SIZE], corner_count, found);
		p_count[i] = corner_count;
		//cvShowImage ("Calibration", src_img[i]);
		//cvWaitKey (0);
	}
	//cvDestroyWindow ("Calibration");

	if (found_num != IMAGE_NUM)
		return -1;
	cvInitMatHeader (&image_points, ALL_POINTS, 1, CV_32FC2, corners);
	cvInitMatHeader (&point_counts, IMAGE_NUM, 1, CV_32SC1, p_count);

	// (5)�����p�����[�^�C�c�݌W���̐���
	cvCalibrateCamera2 (&object_points, &image_points, &point_counts, cvGetSize (src_img[0]), intrinsic, distortion);
	//calibrateCamera (&object_points, &image_points, &point_counts, cvGetSize (src_img[0]), intrinsic, distortion);

	// (6)�O���p�����[�^�̐���
	CvMat sub_image_points, sub_object_points;
	int base = 0;
	cvGetRows (&image_points, &sub_image_points, base * PAT_SIZE, (base + 1) * PAT_SIZE);
	cvGetRows (&object_points, &sub_object_points, base * PAT_SIZE, (base + 1) * PAT_SIZE);
	cvFindExtrinsicCameraParams2 (&sub_object_points, &sub_image_points, intrinsic, distortion, rotation, translation);

	// (7)XML�t�@�C���ւ̏����o��
	CvFileStorage *fs;
	fs = cvOpenFileStorage ("data/result/camera.xml", 0, CV_STORAGE_WRITE);
	cvWrite (fs, "intrinsic", intrinsic);
	cvWrite (fs, "rotation", rotation);
	cvWrite (fs, "translation", translation);
	cvWrite (fs, "distortion", distortion);
	cvReleaseFileStorage (&fs);

	for (i = 0; i < IMAGE_NUM; i++) {
		cvReleaseImage (&src_img[i]);
	}

#endif

#if FIXIMG

	IplImage *src_img, *dst_img;
	CvMat *intrinsic, *distortion;
	CvFileStorage *fs;
	CvFileNode *param;

	// (1)�␳�ΏۂƂȂ�摜�̓ǂݍ���
	if ((src_img = cvLoadImage ("data/images/img (1).jpg", CV_LOAD_IMAGE_COLOR)) == 0)
		return -1;
	dst_img = cvCloneImage (src_img);

	// (2)�p�����[�^�t�@�C���̓ǂݍ���
	fs = cvOpenFileStorage ("data/result/camera.xml", 0, CV_STORAGE_READ);
	param = cvGetFileNodeByName (fs, NULL, "intrinsic");
	intrinsic = (CvMat *) cvRead (fs, param);
	param = cvGetFileNodeByName (fs, NULL, "distortion");
	distortion = (CvMat *) cvRead (fs, param);
	cvReleaseFileStorage (&fs);

	// (3)�c�ݕ␳
	cvUndistort2 (src_img, dst_img, intrinsic, distortion);

	// (4)�摜��\���C�L�[�������ꂽ�Ƃ��ɏI��
	cvNamedWindow ("Distortion", CV_WINDOW_AUTOSIZE);
	cvShowImage ("Distortion", src_img);
	cvNamedWindow ("UnDistortion", CV_WINDOW_AUTOSIZE);
	cvShowImage ("UnDistortion", dst_img);
	cvWaitKey (0);

	cvDestroyWindow ("Distortion");
	cvDestroyWindow ("UnDistortion");
	cvReleaseImage (&src_img);
	cvReleaseImage (&dst_img);
	cvReleaseMat (&intrinsic);
	cvReleaseMat (&distortion);


#endif

#if FIX
	///�c�ݕ␳


	CvMat *intrinsic, *distortion;
	CvFileStorage *fs;
	CvFileNode *param;
	int Xsize=128, Ysize=126;
	//int Xsize=2304, Ysize=1536;
	int numPoint = Xsize*Ysize;
	Mat src(1, numPoint, CV_32FC2);
	Mat dst = Mat::zeros(1, numPoint, CV_32FC2);
	for(int i=0; i<Xsize; i++){
		for(int j=0; j<Ysize; j++){
			src.at<Vec2f>(0,i*Ysize+j)[0] = i;
			src.at<Vec2f>(0,i*Ysize+j)[1] = j;
		}
	}

	// �p�����[�^�t�@�C���̓ǂݍ���
	fs = cvOpenFileStorage ("data/result/camera.xml", 0, CV_STORAGE_READ);
	param = cvGetFileNodeByName (fs, NULL, "intrinsic");
	intrinsic = (CvMat *) cvRead (fs, param);
	param = cvGetFileNodeByName (fs, NULL, "distortion");
	distortion = (CvMat *) cvRead (fs, param);
	cvReleaseFileStorage (&fs);

	//cvMat����Mat�ւ̕ϊ�
	// CvMat -> cv::Mat
	cv::Mat intrinsic_Mat(intrinsic, true); // �f�[�^���R�s�[����
	CV_Assert(intrinsic->data.ptr != intrinsic_Mat.data);
	cv::Mat distortion_Mat(distortion, true); // �f�[�^���R�s�[����
	CV_Assert(distortion->data.ptr != distortion_Mat.data);

	//cout << "src " << src << endl;
	//cout << "dst " << dst << endl;
	cout << "intrinsic_Mat " << intrinsic_Mat << endl;
	cout << "distortion_Mat " << distortion_Mat << endl << endl;

	/// �c�ݕ␳
	undistortPoints(src, dst, intrinsic_Mat, distortion_Mat);
	
	///fx,fy,cx,cy���g���ĉ摜��̓_���W�ɕϊ�
	float fx = intrinsic_Mat.at<float>(0,0);
	float fy = intrinsic_Mat.at<float>(1,1);
	float cx = intrinsic_Mat.at<float>(0,2);
	float cy = intrinsic_Mat.at<float>(1,2);
	Mat dst2 = Mat::zeros(1, numPoint, CV_32FC2);
	for(int i=0; i<Xsize; i++){
		for(int j=0; j<Ysize; j++){
			dst2.at<Vec2f>(0,i*Ysize+j)[0] = dst.at<Vec2f>(0,i*Ysize+j)[0]*fx+cx;
			dst2.at<Vec2f>(0,i*Ysize+j)[1] = dst.at<Vec2f>(0,i*Ysize+j)[1]*fy+cy;
		}
	}

#if 1 //�����摜�J�����@���w�i�@�摜���W�ϊ���
	//���ʂ�`��
	Mat img = cv::Mat::zeros(Xsize*8, Ysize*8, CV_8UC3);
	rectangle(img, cv::Point(0,0), cv::Point(Xsize*8, Ysize*8), cv::Scalar(240,240,240), -1, 4);
	for(int i=0; i<Xsize; i+=4){
		for(int j=0; j<Ysize; j+=4){
			line(img, Point(i*8, j*8), Point(dst2.at<Vec2f>(0,i*Ysize+j)[0]*8, dst2.at<Vec2f>(0,i*Ysize+j)[1]*8), Scalar(0,0,200), 1, 4); 
			ellipse(img, Point(dst2.at<Vec2f>(0,i*Ysize+j)[0]*8, dst2.at<Vec2f>(0,i*Ysize+j)[1]*8), Size(1.5, 1.5), 0, 0, 360, cv::Scalar(0,0,0), -1, 4);
		}
	}
#endif

#if 0 //D3100�@�摜���W�ϊ���
	//���ʂ�`��
	Mat img = cv::Mat::zeros(Ysize/2, Xsize/2, CV_8UC3);
	rectangle(img, cv::Point(0,0), cv::Point(Xsize/2, Ysize/2), cv::Scalar(240,240,240), -1, 4);
	for(int i=0; i<Ysize; i+=40){
		for(int j=0; j<Xsize; j+=40){
			line(img, Point(j/2, i/2), Point(dst2.at<Vec2f>(0,i*Xsize+j)[0]/2, dst2.at<Vec2f>(0,i*Xsize+j)[1]/2), Scalar(0,0,200), 1, 4); 
			ellipse(img, Point(dst2.at<Vec2f>(0,i*Xsize+j)[0]/2, dst2.at<Vec2f>(0,i*Xsize+j)[1]/2), Size(1.5, 1.5), 0, 0, 360, cv::Scalar(0,0,0), -1, 4);
		}
	}
#endif

	cv::namedWindow("�␳�x�N�g��", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
	cv::imshow("�␳�x�N�g��", img);
	cv::waitKey(0);
	
	//���f�[�^
	//���ʂ�CSV�ɕۑ�
	//�t�@�C�������o��
	FILE *outfp;
	char outfilename[100];
	sprintf(outfilename,"data/result/output_row.csv");
	outfp = fopen(outfilename,"w");
	if(outfp == NULL){
		printf("%s�t�@�C�����J���܂���\n",outfilename);
		return -1;
	}
	for(int i=0; i<Xsize; i++){
		for(int j=0; j<Ysize; j++){
			fprintf(outfp,"%f %f\n", dst.at<Vec2f>(0,i*Ysize+j)[0],dst.at<Vec2f>(0,i*Ysize+j)[1] );
		}
	}
	fclose(outfp);

	//�摜���W�ϊ���
	//���ʂ�CSV�ɕۑ�
	//�t�@�C�������o��
	FILE *outfp2;
	char outfilename2[100];
	sprintf(outfilename2,"data/result/output.csv");
	outfp2 = fopen(outfilename2,"w");
	if(outfp2 == NULL){
		printf("%s�t�@�C�����J���܂���\n",outfilename2);
		return -1;
	}
	for(int i=0; i<Xsize; i++){
		for(int j=0; j<Ysize; j++){
			fprintf(outfp2,"%f %f\n", dst2.at<Vec2f>(0,i*Ysize+j)[0],dst2.at<Vec2f>(0,i*Ysize+j)[1] );
		}
	}
	fclose(outfp2);

#endif

	return 0;
}