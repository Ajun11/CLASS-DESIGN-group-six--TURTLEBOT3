/*
 * @Descripttion:
 * @version:
 * @Author: liujun
 * @Date: 2022-05-17 01:33:27
 * @LastEditors: liujun
 * @LastEditTime: 2022-05-17 19:24:18
 */
//**C++Stl
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <map>
#include <string>
//**opencv lib
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
//**ros need
#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<geometry_msgs/Twist.h> 
using namespace cv;
using namespace dnn;
using namespace std;
string  Yolo_Base =  "/home/liujun/Code/wangfei_slam/catkin_cars/src/image_yolo/include/";
string  Yolo_Cfg = Yolo_Base + "yolo-fastest/yolo-fastest-xl.cfg";
string  Yolo_Weights = Yolo_Base + "yolo-fastest/yolo-fastest-xl.weights";
string  Yolo_Coconame = Yolo_Base + "coco.names";
struct Net_config
{
	float confThreshold; // Confidence threshold
	float nmsThreshold;  // Non-maximum suppression threshold
	int inpWidth;  // Width of network's input image
	int inpHeight; // Height of network's input image
	string classesFile;
	string modelConfiguration;
	string modelWeights;
	string netname;
};
Net_config yolo_nets[4] = {
	{0.5, 0.4, 416, 416,"coco.names", "yolov3/yolov3.cfg", "yolov3/yolov3.weights", "yolov3"},
	{0.5, 0.4, 608, 608,"coco.names", "yolov4/yolov4.cfg", "yolov4/yolov4.weights", "yolov4"},
	{0.5, 0.4, 320, 320, Yolo_Coconame, Yolo_Cfg, Yolo_Weights, "yolo-fastest"},
	{0.5, 0.4, 320, 320,"coco.names", "yolobile/csdarknet53s-panet-spp.cfg", "yolobile/yolobile.weights", "yolobile"}
};
class IMAGE_HANDLER//����Ϊ����д
{
public:
	void IMAGE_HANDLER_INIT(int *Points,vector<Rect>Boxaxis)
	{
		//*��������
		Pointer = Points;
		copy(Boxaxis.begin(), Boxaxis.end(), inserter(Box_axis, Box_axis.begin()));
		//*����������
		SizeOfImageVector = 0;
		Position = 0;
		Max_Index = 0;
		Max_Value = 0;
		Judge_Over = false;
		PointSums.clear();
		MembersSum.clear();
	}
	void Image_Defulter_Runner(vector<Mat> ImageVector)
	{
		vector<vector<Point>>contours;
		vector<Vec4i>hierachy;
		std::vector<cv::Point> tempPoint;
		Mat images, img_gray, mean, thresh_low, img2;//�ֱ��Ӧ���յ��ĵ�һ��ͼ�񣬻ҶȻ�ͼ����ָ�˲���ѡ�ĸ�
		Mat element = getStructuringElement(MORPH_ELLIPSE, Size(20, 20), Point(-1, -1));//���������
		Mat element1 = getStructuringElement(MORPH_ELLIPSE, Size(40, 40), Point(-1, -1));//���������
		SizeOfImageVector = ImageVector.size();
		for (int i = 0; i < SizeOfImageVector; i++)
		{
			// �㼯
			tempPoint.clear();
			images = ImageVector[i];
			cvtColor(images, img_gray, COLOR_BGR2GRAY);//灰度化
			medianBlur(img_gray, mean, 5);
			Canny(mean, img2, 150, 100, 3);
			findContours(img2, contours, hierachy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

			for (size_t t = 0; t < contours.size(); t++)
			{
				drawContours(images, contours, -1, Scalar(0, 0, 255), 2);
			}
			imshow("images after",images);
			for (int k = 0; k < contours.size(); k++)
			{
				for (int m = 0; m < contours[k].size(); m++)
				{
					tempPoint.push_back(contours[k][m]);
				}
			}
			Max_Min_Value(tempPoint, ImageVector[i]);
		}
		if (Judge_Over)
		{
			cout << "The aim is that:" << Max_Index << endl;
			map<int, vector<int>>::iterator iter;
			if(Max_Value!=0)
			{
				for (int i = 0; i < MembersSum.size(); i++)
			{
				iter = PointSums.find(MembersSum[i]);
				/*ImageVector[Max_Index].at<Vec3b>(iter->second[0], MembersSum[i]) = (255, 255, 255);
				ImageVector[Max_Index].at<Vec3b>(iter->second[1], MembersSum[i]) = (255, 255, 255);*/
				*(Pointer + i * 3) = iter->first+ Box_axis[Max_Index].x;
				*(Pointer + i * 3 + 1) = iter->second[0]+ Box_axis[Max_Index].y;
				*(Pointer + i * 3 + 2) = iter->second[1]+ Box_axis[Max_Index].y;
			}
			}
		}
	}
	int Max_Min_Value(std::vector<cv::Point> SpecialPoint, cv::Mat ImageForRedFind)//��������ÿһ��Ŀ��ķ�Χ
	{
		int Red_Return;
		map<int, vector<int>> PointHandle;
		map<int, vector<int>> PointLast;
		map<int, vector<int>>::iterator iter;
		vector<int> MembersIndex;
		for (int i = 0; i < SpecialPoint.size(); i++)//��xԪ�ض�Ӧ��yԪ�ض���������
		{
			iter = PointHandle.find(SpecialPoint.at(i).x);
			if (iter != PointHandle.end())
			{
				PointHandle[SpecialPoint.at(i).x].push_back(SpecialPoint.at(i).y);
			}
			else
			{
				MembersIndex.push_back(SpecialPoint.at(i).x);//��Ϊ��������������ֵ
				PointHandle[SpecialPoint.at(i).x].push_back(SpecialPoint.at(i).y);
			}
		}
		for (int i = 0; i < MembersIndex.size(); i++)
		{
			int max = 0, min = 1000;
			iter = PointHandle.find(MembersIndex[i]);
			for (int j = 0; j < iter->second.size(); j++)
			{
				if (iter->second[j] > max)
				{
					max = iter->second[j];
				}
				if (iter->second[j] < min)
				{
					min = iter->second[j];
				}
			}
			PointLast[MembersIndex[i]].push_back(min);//���µ������洢��x��Ӧ��y�������Сֵ
			PointLast[MembersIndex[i]].push_back(max);
		}
		Image_Red_find(PointLast, MembersIndex, ImageForRedFind);
	}
	void Image_Red_find(map<int, vector<int>> Xycontainer, vector<int> Xyindex, cv::Mat ForMe)
	{
		map<int, vector<int>>::iterator iter;
		int RedPointNums,BluePointNums, Color_B, Color_G, Color_R, Point_Sums;
		RedPointNums = 0;
		BluePointNums = 0;
		for (int i = 0; i < Xyindex.size(); i++)
		{
			iter = Xycontainer.find(Xyindex[i]);
			for (int loop2 = iter->second[0]; loop2 <= iter->second[1]; loop2++)
			{
				Color_B = ForMe.at<Vec3b>(loop2, Xyindex[i])[0];
				Color_G = ForMe.at<Vec3b>(loop2, Xyindex[i])[1];
				Color_R = ForMe.at<Vec3b>(loop2, Xyindex[i])[2];
				if (Color_B >= 5 && Color_B <= 8 && Color_G >= 9 && Color_G <= 11 && Color_R >= 24 && Color_R <= 33)//衣服
				{
					RedPointNums++;
				}
				if(Color_B >= 34 && Color_B <= 40 && Color_G >= 8 && Color_G <= 13 && Color_R >= 3 && Color_R <= 8)//裤子
				{
					BluePointNums++;
				}
			}
		}
		if (Position < SizeOfImageVector)
		{
			if (Max_Value < (RedPointNums+BluePointNums))
			{
				MembersSum.clear();//�Դ洢���ֵ������������գ��洢�µ�
				PointSums.clear();
				copy(Xycontainer.begin(), Xycontainer.end(), inserter(PointSums, PointSums.begin()));
				copy(Xyindex.begin(), Xyindex.end(), inserter(MembersSum, MembersSum.begin()));
				Max_Value = RedPointNums+BluePointNums;
				Max_Index = Position;
			}
			Position++;
		}
		if (Position == SizeOfImageVector)
		{
			Judge_Over = true;
		}
	}
private:
	int                                SizeOfImageVector;
	map<int, vector<int>>			   PointSums;
	vector<int>			               MembersSum;
	vector<Rect>                       Box_axis;
	int								   Position;
	int								   Max_Index;
	int								   Max_Value;
	int								   *Pointer;
	bool                               Judge_Over;
};

class YOLO
{
public:
	YOLO(Net_config config);
	void detect(Mat& frame);
	void YOLO_Runner(int *Points);
private:
	float confThreshold;
	float nmsThreshold;
	bool  DetectOk;
	int inpWidth;
	int inpHeight;
	char netname[20];
	vector<string> classes;
	vector<Mat>    Imagestorage;//ͼ��Ĵ洢
	vector<Rect>   Boxstorage;//����洢
	Net net;
	void postprocess(Mat& frame, const vector<Mat>& outs);
	void drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame);
};
void YOLO::YOLO_Runner(int *Points)//���д˺���ǰ����Ҫ����YOLO::Detect��������ʶ�𣬴˺���Ϊ����IMAGE_HANDLER���ͼ��������
{
	IMAGE_HANDLER runner;
	if(DetectOk)
	{
		runner.IMAGE_HANDLER_INIT(Points,Boxstorage);
		runner.Image_Defulter_Runner(Imagestorage);
	}
	Imagestorage.clear();
	Boxstorage.clear();
	DetectOk = false;
}

YOLO::YOLO(Net_config config)
{
	cout << "net use " << config.netname << endl;
	this->confThreshold = config.confThreshold;
	this->nmsThreshold = config.nmsThreshold;
	this->inpWidth = config.inpWidth;
	this->inpHeight = config.inpHeight;
	strcpy(this->netname, config.netname.c_str());

	ifstream ifs(config.classesFile.c_str());
	string line;
	while (getline(ifs, line)) this->classes.push_back(line);

	this->net = readNetFromDarknet(config.modelConfiguration, config.modelWeights);
	this->net.setPreferableBackend(DNN_BACKEND_OPENCV);
	this->net.setPreferableTarget(DNN_TARGET_CPU);
}

void YOLO::postprocess(cv::Mat& frame, const vector<Mat>& outs)   // remove the bounding boxes with low confidence using non-maxima suppression
{
	vector<int> classids;
	vector<float> confidences;
	vector<Rect> boxes;
	for (size_t i = 0; i < outs.size(); ++i)
	{
		// scan through all the bounding boxes output from the network and keep only the
		// ones with high confidence scores. assign the box's class label as the class
		// with the highest score for the box.
		float* data = (float*)outs[i].data;
		for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
		{
			Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
			Point classidpoint;
			double confidence;
			// get the value and location of the maximum score
			minMaxLoc(scores, 0, &confidence, 0, &classidpoint);
			if (confidence > this->confThreshold)
			{
				int centerx = (int)(data[0] * frame.cols);
				int centery = (int)(data[1] * frame.rows);
				int width  =  (int)(data[2] * frame.cols);
				int height =  (int)(data[3] * frame.rows);
				int left = centerx - width / 2;
				int top = centery - height / 2;
				classids.push_back(classidpoint.x);
				confidences.push_back((float)confidence);
				boxes.push_back(Rect(left, top, width, height));
			}
		}
	}

	// perform non maximum suppression to eliminate redundant overlapping boxes with
	// lower confidences
	vector<int> indices;
	Mat roi;
	int y_max,x_max;
	NMSBoxes(boxes, confidences, this->confThreshold, this->nmsThreshold, indices);
	for (size_t i = 0; i < indices.size(); ++i)
	{
		Mat tem_frame = frame.clone();
		int idx = indices[i];
		Rect box = boxes[idx];
		this->drawPred(classids[idx], confidences[idx], box.x, box.y,
			box.x + box.width, box.y + box.height, frame);
		y_max=box.y + box.height;
		x_max=box.x+box.width;
		//*得到的裁剪值有时会越界，限制一下，不然会崩溃
		if(box.y<0)
		{
		box.y=0;
		}
		if(box.x<0)
		{
		box.x=0;
		}
		if (y_max>frame.rows)
		{
		  y_max=frame.rows;
		}
		if (x_max>frame.cols)
		{
		 x_max=frame.cols;
		}
		roi = tem_frame(Range(box.y ,y_max ), Range(box.x , x_max));
		Imagestorage.push_back(roi);
		Boxstorage.push_back(box);
	}
	DetectOk = true;
	/*imshow("ROI", storage[0]);*/
	//cv::resize(storage[0], storage[0], cv::size(), 8, 4)
}

void YOLO::drawPred(int classid, float conf, int left, int top, int right, int bottom, Mat& frame)   // draw the predicted bounding box
{
	//draw a rectangle displaying the bounding box
	rectangle(frame, Point(left, top), Point(right, bottom), Scalar(0, 0, 255), 3);
	//get the label for the class name and its confidence
	string label = format("%.2f", conf);
	if (!this->classes.empty())
	{
		CV_Assert(classid < (int)this->classes.size());
		label = this->classes[classid] + ":" + label;
	}

	//display the label at the top of the bounding box
	int baseline;
	Size labelsize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
	top = max(top, labelsize.height);
	//rectangle(frame, point(left, top - int(1.5 * labelsize.height)), point(left + int(1.5 * labelsize.width), top + baseline), scalar(0, 255, 0), filled);
	putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 255, 0), 1);
}

void YOLO::detect(Mat& frame)
{
	Mat blob;
	blobFromImage(frame, blob, 1 / 255.0, Size(this->inpWidth, this->inpHeight), Scalar(0, 0, 0), true, false);
	this->net.setInput(blob);
	vector<Mat> outs;
	this->net.forward(outs, this->net.getUnconnectedOutLayersNames());
	this->postprocess(frame, outs);

	vector<double> layerstimes;
	double freq = getTickFrequency() / 1000;
	double t = net.getPerfProfile(layerstimes) / freq;
	string label = format("%s inference time : %.2f ms", this->netname, t);
	putText(frame, label, Point(0, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
	//imwrite(format("%s_out.jpg", this->netname), frame);
}
