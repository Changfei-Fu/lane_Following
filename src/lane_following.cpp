#include "lane_following.hpp"
#include <geometry_msgs/Twist.h>
#include <memory>
#include <time.h>
#include <ros/ros.h>
#include "opencv2/core/eigen.hpp"


using namespace cv;

cv::Mat Lane_Following::Homography(){
    double fx = 160/tan(1.085/2);
    double d=0.115;
    Eigen::Matrix<double, 3, 3> K;
    Eigen::Matrix<double, 3, 3> K_I;
    Eigen::Matrix<double, 3, 3> R;
    Eigen::Matrix<double, 3, 3> H;
    Eigen::Matrix<double, 3, 1> t;
    Eigen::Matrix<double, 3, 1> n;
    K<<     fx, 0,  160,
            0,  fx, 120,
            0,  0,  1;

    R<<     1, 0,  0,
            0, 0,  1,
            0, -1, 0;

    t << 0,0.9, 0.5;

    n << 0, -1, 0;

    H = K * (R.transpose() - t*n.transpose()/d) * K.inverse();
    Mat Homography;
    cv::eigen2cv(H,Homography);
    return Homography;
}



Lane_Following::Lane_Following(){
    ros::NodeHandle nh;
    img_sub = nh.subscribe("/camera/image", 10, &Lane_Following::subs_callback, this);
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
    ros::Rate loop_rate(100);
    geometry_msgs::Twist cmd_vel;
    while(ros::ok())
    {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.75;
	//error2=log(error2);
	if(abs(error2)>30){
		cmd_vel.linear.x = 0.35;
	}

	cmd_vel.angular.z = (error2*90.0/160)/15+3*(error2-error1);
/*	if(abs(cmd_vel.angular.z)>6){
		cmd_vel.angular.z*=0.8;
	}*/
        error1=error2;

        cmd_vel_pub_.publish(cmd_vel);

        ros::spinOnce();
        loop_rate.sleep();
    }

}

void Lane_Following::subs_callback(const sensor_msgs::ImageConstPtr& msg) {

    Mat dst;
    frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    //gray = cv_bridge::toCvShare(msg, "mono8")->image;
    Mat gray;
    Mat image;
    cv::warpPerspective(frame,image,Lane_Following::Homography(),frame.size());
    cv::cvtColor(image,gray,CV_BGRA2GRAY);
    threshold(gray, gray, 80, 255, THRESH_BINARY);

    dst = gray(Rect(30, gray.rows * 2/3, 290, gray.rows *1/ 3));

    Mat yellow_image=Mat::zeros(gray.rows,gray.cols,CV_8U);
    Mat white_image=Mat::zeros(gray.rows,gray.cols,CV_8U);
    
    for(int i=0;i<image.rows;i++){
        auto* data = image.ptr<uchar>(i);
        auto* gray_data = gray.ptr<uchar>(i);
        auto* yellow_data = yellow_image.ptr<uchar>(i);
        auto* white_data = white_image.ptr<uchar>(i);

        for(int j=0;j<gray.cols;j++){
            if(int(gray_data[j])!=0){
                int H,S,V;
                int B=int(data[3*j]);
                int G=int(data[3*j+1]);
                int R=int(data[3*j+2]);

                int Max,Min;
                if(R>G&&R>B){
                    Max=R;
                }else if(G>B){
                    Max=G;
                }else{
                    Max=B;
                }
                if(R<G&&R<B){
                    Min=R;
                }else if(G>B){
                    Min=B;
                }else{
                    Min=G;
                }
                V=Max;
                if(V==0){
                    continue;
                }
                if(Max==Min){
                    H=0;
                }else{
                    if( R == Max){
                        H = (G-B)/(Max-Min);
                    }else if( G == Max){
                        H = 2 + (B-R)/(Max-Min);
                    }else if(B == Max){
                        H = 4 + (R-G)/(Max-Min);
                    }
                }
                H*=30;
                if(H<0){
                    H+=180;
                }
                S=255*(Max-Min)/Max;

                if(H>=26&&H<=34){
                    if(S>=43&&V>=46){
                        yellow_data[j]=255;
                    }
                } else if(S>=0&&S<=30&&V>150){
                    white_data[j]=255;
                }
            }
        }
    }

    /*Mat yellow_labels, yellow_stats, yellow_centroids;
    Mat white_labels, white_stats, white_centroids;*/
    cv::Moments M1 = cv::moments(yellow_image);
    cv::Moments M2 = cv::moments(white_image);

    if(M1.m00>0){
        int cx1 = int(M1.m10/M1.m00);
        int cy1 = int(M1.m01/M1.m00);
        int cx2 = int(M2.m10/M2.m00);
        int cy2 = int(M2.m01/M2.m00);

        int fpt_x = (cx1 + cx2)/2;
        int fpt_y = (cy1 + cy2)/2 + 2*image.rows/3;

        cv::circle(image, cv::Point(cx1, cy1), 10, (0,0,255), -1);
        cv::circle(image, cv::Point(cx2, cy2), 10, (0,255,0), -1);
        cv::circle(image, cv::Point(fpt_x, fpt_y), 10, (0,0,0), -1);
        error2 = image.cols/2 - fpt_x;
	

    }
	imshow("image",image);
    waitKey(1);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "lanefollowing");
    Lane_Following lf;
    return 0;
}
