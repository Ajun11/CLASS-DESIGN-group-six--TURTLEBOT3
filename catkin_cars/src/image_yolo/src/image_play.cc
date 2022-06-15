/*
 * @Descripttion: 
 * @version: 
 * @Author: liujun
 * @Date: 2022-05-07 11:17:40
 * @LastEditors: liujun
 * @LastEditTime: 2022-05-07 15:00:37
 */

#include<yolo.h>
//**
//**彩色图像话题
#define RGB_DEFAULT_TOPIC              "/camera/rgb/image_raw"
//**深度图像话题
#define DEPTH_DEFAULT_TOPIC            "/camera/depth/image_raw"
//**彩色相机内参话题
#define RGB_CAMERA_INFO_TOPIC          "/camera/rgb/camera_info"
//**深度相机内参话题
#define DEPTH_CAMERA_INFO_TOPIC        "/camera/depth/camera_info"
//**Turtlebot3速度话题
#define TURTLEBOT3_CONTROL_TOPIC       "/cmd_vel"
class Image_resolution
{
    public :
    Image_resolution(int argc,char**argv,const char *Node_name)//析构函数，感谢学长
        {
            ros::init(argc,argv,Node_name);
            ros::NodeHandle cars("cars_play");
            RGB_IMG=cars.subscribe(RGB_DEFAULT_TOPIC,1,&Image_resolution::RGB_img_CallBack,this);
            DEPTH_IMG =cars.subscribe(DEPTH_DEFAULT_TOPIC,1,&Image_resolution::Depth_img_CallBack,this);
            RGB_CAMERA_INFO = cars.subscribe(RGB_CAMERA_INFO_TOPIC, 1, &Image_resolution::RGB_Camera_info_CallBack, this);
            DEPTH_CAMERA_INFO = cars.subscribe(DEPTH_CAMERA_INFO_TOPIC, 1, &Image_resolution::Depth_Camera_info_CallBack, this);
            Cars_control = cars.advertise<geometry_msgs::Twist>(TURTLEBOT3_CONTROL_TOPIC, 1);
            ros::Duration(1.0).sleep();
            ROS_INFO("Everything is Ok");
        }
    ~Image_resolution(){ }
    void run(void) 
    {
        ros::spin();
    }
    public:
        struct Camera_info_paras
        {
            double fx, fy, cx,cy;
            bool   isOk;
        };
        struct Camera_Rect_Info
        {
            int x_basis,x_top,y_basis,y_top,judge_level;
            
        };
        struct Cars_PID_Control{
            float kp, Ki, Kd;
            float error_now,last_error, last_error_2;
        };

    public:
        void RGB_img_CallBack(const sensor_msgs::ImageConstPtr& msg)
        {
            cv::Mat images;
            YOLO   yolo_model(yolo_nets[2]);
            memset(SuperArray, '\0', sizeof(SuperArray));//**检测前清空检测数组
            try
            {
                images=cv_bridge::toCvShare(msg,"bgr8")->image;
            }
            catch(cv_bridge::Exception)
            {
               ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
            }
            cv::resize(images, lasted_rgb, cv::Size(), scale_factor, scale_factor);
            yolo_model.detect(lasted_rgb);
		    yolo_model.YOLO_Runner(SuperArray);
            if(SuperArray[0]!='\0')
            {
                 Yolo_Ready = true;
            }
            else
            {
                Yolo_Ready = false;
            }
            for (int i = 0; i < 1024; i++)
            {
                if (SuperArray[i*3] == '\0')
                    break;
                else
                {
                    lasted_rgb.at<Vec3b>(SuperArray[i * 3 + 1], SuperArray[i * 3]) = (0, 0, 255);
                    lasted_rgb.at<Vec3b>(SuperArray[i * 3 + 2], SuperArray[i * 3]) = (0, 0, 255);
                }
            }
            cv::imshow("After Yolo Handler", lasted_rgb);
            cv::waitKey(30);
        }
        void Depth_img_CallBack(const sensor_msgs::ImageConstPtr& msg)
        {
            cv::Mat images;
            try
            {
                images=cv_bridge::toCvShare(msg)->image;
            }
            catch(cv_bridge::Exception)
            {
               ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
            }
            cv::resize(images,lasted_depth,cv::Size(),scale_factor,scale_factor);
            cv::Mat imgcoloredDepth;
            ColoredDepth(lasted_depth, imgcoloredDepth);
            //cv::imshow("Depth image",imgcoloredDepth);
            if((depth_camera_inside.isOk&&Yolo_Ready)==1)//深度相机内参获得完毕，彩色图像处理完毕
            {
                ROS_INFO_STREAM("Rectangle success , please wait......");
                Person_Distance_Get_Void(depth_camera_inside);
                //Box_Distance_Get_void(depth_camera_inside);
                Yolo_Ready = false;
            }
            else
            {
                ROS_ERROR("May be aim is out of your camera,please wait for controlers,don't be hurry!!!");
                cars_speed.linear.x=0;
                cars_speed.angular.z=1;
                Cars_control.publish(cars_speed);
            }
            cv::waitKey(10);
        }
        void RGB_Camera_info_CallBack(const sensor_msgs::CameraInfo::ConstPtr& pmsg)
        {
            rgb_camera_inside.fx = pmsg->K[0] * scale_factor;
            rgb_camera_inside.fy = pmsg->K[4] * scale_factor;
            rgb_camera_inside.cx = pmsg->K[2] * scale_factor;
            rgb_camera_inside.cy = pmsg->K[5] * scale_factor;
            rgb_camera_inside.isOk = true;
            if(rgb_camera_inside.isOk)
            {
                ROS_INFO_STREAM("RGB Camera params：\n" << "fx:=" << rgb_camera_inside.fx 
                                                << "fy:=" << rgb_camera_inside.fy
                                                << "cx:=" << rgb_camera_inside.cx 
                                                << "cy:=" << rgb_camera_inside.cy);
            }
            RGB_CAMERA_INFO.shutdown();//得到内参取消订阅
        }
        void Depth_Camera_info_CallBack(const sensor_msgs::CameraInfo::ConstPtr& pmsg)
        {
            depth_camera_inside.fx = pmsg->K[0] * scale_factor;
            depth_camera_inside.fy = pmsg->K[4] * scale_factor;
            depth_camera_inside.cx = pmsg->K[2] * scale_factor;
            depth_camera_inside.cy = pmsg->K[5] * scale_factor;
            depth_camera_inside.isOk = true;
            if(rgb_camera_inside.isOk)
            {
                ROS_INFO_STREAM("Depth Camera params：\n" << "fx:=" << depth_camera_inside.fx 
                                                << "fy:=" << depth_camera_inside.fy
                                                << "cx:=" << depth_camera_inside.cx 
                                                << "cy:=" << depth_camera_inside.cy);
            }
            DEPTH_CAMERA_INFO.shutdown();//得到内参取消订阅
        }
        void Box_Distance_Get_void(Image_resolution::Camera_info_paras Gets)
        {
            float x=0,y=0,z=0;
            float axis_symbols;
            int nums_profit=0;
            int cols,rows;
            for(cols =Rectangle.x_basis+(int)((Rectangle.x_top-Rectangle.x_basis)/4);
            cols<=Rectangle.x_top-(int)((Rectangle.x_top-Rectangle.x_basis)/4);cols++)
            {
                for(rows=Rectangle.y_basis+(int)((Rectangle.y_top-Rectangle.y_basis)/4);
                rows<=Rectangle.y_top-(int)((Rectangle.y_top-Rectangle.y_basis)/4);rows++)
                {
                    axis_symbols=lasted_depth.at<float>(rows, cols);
                    if(std::isnan(axis_symbols))
                    {
                        continue;
                    }
                    else
                   {
                    z +=  axis_symbols;
                    x += (axis_symbols / Gets.fx) * (cols - Gets.cx);
                    y += (axis_symbols / Gets.fy) * (rows - Gets.cy);
                    nums_profit++;
                  }
                }
            }
            x=x/nums_profit;
            y=y/nums_profit;
            z=z/nums_profit;
            ROS_INFO_STREAM("The x_aixs and y_axis and z_axis is："<<std::endl <<"x_bias:"<< x << "y_bias:" << y << 
                                                                            "z_bias:" << z<<"nums is "<<nums_profit);
            if(std::isfinite(x)&&std::isfinite(z))
            {
                ROS_INFO_STREAM("Now the car is too far ,we will move straight first!!!");
                x = 0;
                z = 1.6;
            }
            PID_Controler(x,z);
        }
        void Person_Distance_Get_Void(Image_resolution::Camera_info_paras Gets)
        {
            float x=0,y=0,z=0;
            float axis_symbols;
            int nums_profit=0,nums_nodeep=0;
            int x_index,cols,rows;
            x_index=0,cols=0,rows=0;
            while(true)
            {
                if(SuperArray[3*x_index]=='\0')
                    break;
                else
                {
                    cols=SuperArray[3*x_index];
                    for(rows=SuperArray[3*x_index+1];rows<=SuperArray[3*x_index+2];rows++)
                    {
                        axis_symbols=lasted_depth.at<float>(rows,cols);
                        if(std::isnan(axis_symbols))
                        {
                            nums_nodeep++;
                            continue;
                        }
                        else
                        {
                            z+=axis_symbols;
                            x += (axis_symbols / Gets.fx) * (cols - Gets.cx);
                            y += (axis_symbols / Gets.fy) * (rows - Gets.cy);
                            nums_profit++;
                        }
                    }
                }
                x_index++;
            }
            x=x/nums_profit;
            y=y/nums_profit;
            z=z/nums_profit;
            ROS_INFO_STREAM("The x_aixs and y_axis and z_axis is："<<std::endl <<"x_bias:"<< x << "y_bias:" << y << 
                                                                            "z_bias:" << z<<"nums is "<<nums_profit<<"nums_nodeep"<<nums_nodeep);
            if(std::isnan(x)&&std::isnan(z))
            {
                ROS_INFO_STREAM("Now the car is too far ,we will move straight first!!!");
                x = 0;
                z = 3;
            }
            PID_Controler(x,z);
        }
        void PID_Controler(float x_error,float y_error)
        {
            float angle_send, linear_send;
            cars_pid_linear_paras.error_now = y_error;
            cars_pid_angles_paras.error_now = atan(double(x_error/y_error));
            linear_send = cars_pid_linear_paras.kp * (cars_pid_linear_paras.error_now - cars_pid_linear_paras.last_error) +
                          cars_pid_linear_paras.Ki * (cars_pid_linear_paras.error_now) + cars_pid_linear_paras.Kd * (
                          cars_pid_linear_paras.error_now -cars_pid_linear_paras.last_error * 2 + cars_pid_linear_paras.last_error_2);
            cars_pid_linear_paras.last_error_2 = cars_pid_linear_paras.last_error;
            cars_pid_linear_paras.last_error = cars_pid_linear_paras.error_now;
            if(linear_send>=0.22)
                linear_send = 0.22;
            angle_send = cars_pid_angles_paras.kp * (cars_pid_angles_paras.error_now - cars_pid_angles_paras.last_error) +
                         cars_pid_angles_paras.Ki * (cars_pid_angles_paras.error_now) + cars_pid_angles_paras.Kd * (
                         cars_pid_angles_paras.error_now - cars_pid_angles_paras.last_error * 2 + cars_pid_angles_paras.last_error_2);
            if(angle_send>=0.3)
                angle_send = 0.3;
            else if(angle_send<=-0.3)
                angle_send = -0.3;
            if( cars_pid_angles_paras.error_now<=0.1&&cars_pid_angles_paras.error_now>=-0.1)
            {
                angle_send = 0;
            }
            if(cars_pid_linear_paras.error_now<=1.5)
            {
                linear_send = 0;
            } 
            cars_speed.linear.x = linear_send;
            // cout<<"anfle_send"<<angle_send<<"error is"<<cars_pid_angles_paras.error_now<<endl;
            cars_speed.angular.z = angle_send*-1;
	        cout<<"angle_now"<<cars_speed.angular.z<<endl;
            Cars_control.publish(cars_speed);
        }
    private:
            void ColoredDepth(const cv::Mat& imgSrc, cv::Mat& imgDst)//学长的深度图像处理函数，会变得很美
            {
                cv::Mat imgAbs;
                cv::convertScaleAbs(imgSrc, imgAbs, 51.2, 0.0);
                cv::applyColorMap(255 - imgAbs, imgDst, cv::COLORMAP_JET);
                size_t nRows = imgSrc.rows;
                size_t nCols = imgSrc.cols;
                cv::Vec3b cvBlack(0, 0, 0);
                for(size_t nIdY = 0; nIdY < nRows; ++nIdY)
                {
                    for(size_t nIdX = 0; nIdX < nCols; ++nIdX)
                    {
                        if(std::isnan(imgSrc.at<float>(nIdY, nIdX)))
                        {
                            imgDst.at<cv::Vec3b>(nIdY, nIdX) = cvBlack;
                        }
                    }
                }
            }

    private:
            ros::Subscriber                                RGB_IMG;
            ros::Subscriber                                DEPTH_IMG;
            ros::Subscriber                                RGB_CAMERA_INFO;
            ros::Subscriber                                DEPTH_CAMERA_INFO;
            ros::Publisher                                 Cars_control;
            Image_resolution::Camera_info_paras            rgb_camera_inside;
            Image_resolution::Camera_info_paras            depth_camera_inside;
	        Image_resolution::Camera_Rect_Info		       Rectangle;
            geometry_msgs::Twist                           cars_speed;
            Image_resolution::Cars_PID_Control             cars_pid_linear_paras = {0.25, 0.5, 0.5, 0, 0, 0};
            Image_resolution::Cars_PID_Control             cars_pid_angles_paras = {0.25, 0.5, 0.5, 0, 0, 0};
            bool                                           Yolo_Ready;
            cv::Mat                                        lasted_rgb;
            cv::Mat                                        lasted_depth;
            int                                            pointer[5];
            int                                            SuperArray[3072];
            float                                          scale_factor=0.5;
};
int main(int argc,char **argv)
{
    Image_resolution node(argc,argv,"cars_controler");
    node.run();
    return 0;
}
