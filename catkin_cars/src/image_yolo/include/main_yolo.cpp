#include "yolo.h"
int main()
{
	YOLO yolo_model(yolo_nets[2]);
	string imgpath = "c:/users/lj/desktop/wangfei/class_two_opencv/yolov34-cpp-opencv-dnn-master/dfc.mp4";
	VideoCapture capture(imgpath);
	int SuperArray[3072];
	vector<vector<Point>>contours;
	/*Mat srcimg = imread(imgpath);
	cv::resize(srcimg, srcimg, cv::Size(), 1, 1);
	yolo_model.detect(srcimg);
	imshow("srcimage", srcimg);
	waitKey(0);*/
	while (1)
	{
		Mat frame;//定义一个mat变量，用于存储每一帧的图像
		capture >> frame;  //读取当前帧
		//若视频播放完成，退出循环
		if (frame.empty())
		{

			break;
		}
		cv::resize(frame, frame, cv::Size(), 0.5, 0.5);
		yolo_model.detect(frame);
		yolo_model.YOLO_Runner(SuperArray);
		/*int nums_length;
		for (int i = 0; i < 3072; i++)
		{
			if (SuperArray[i] == '\0')
			{
				nums_length = i;
				break;
			}
		}
		cout << nums_length << endl;*/
		for (int i = 0; i < 1024; i++)
		{
			if (SuperArray[i*3] == '\0')
				break;
			else
			{
				frame.at<Vec3b>(SuperArray[i * 3 + 1], SuperArray[i * 3]) = (0, 0, 255);
				frame.at<Vec3b>(SuperArray[i * 3 + 2], SuperArray[i * 3]) = (0, 0, 255);
			}
		}
		memset(SuperArray, '\0', sizeof(SuperArray));
		imshow("读取视频", frame);  //显示当前帧
		waitKey(30);  //延时30ms
	}
	return 0;
	//static const string kwinname = "deep learning object detection in opencv";
	//namedwindow(kwinname, window_normal);
	//imshow(kwinname, srcimg);
	//waitkey(0);
	//destroyallwindows();
}