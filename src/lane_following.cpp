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

    t << 0,0.3, 0.4;

    n << 0, -1, 0;

    H = K * (R.transpose() - t*n.transpose()/d) * K.inverse();
    Mat Homography;
    cv::eigen2cv(H,Homography);
    return Homography;
}



Lane_Following::Lane_Following(){
    ros::NodeHandle nh;
    img_sub = nh.subscribe("/camera/image/compressed", 10, &Lane_Following::subs_callback, this);
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
    ros::Rate loop_rate(100);
    geometry_msgs::Twist cmd_vel;
    while(ros::ok())
    {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.15;
	//error2=log(error2);
	if(abs(error2)>30){
		cmd_vel.linear.x = 0.05;
	}

	cmd_vel.angular.z = 3*(error2*90.0/160)/15+3*(error2-error1);
/*	if(abs(cmd_vel.angular.z)>6){
		cmd_vel.angular.z*=0.8;
	}*/
        error1=error2;

        cmd_vel_pub_.publish(cmd_vel);
        //cout<<cmd_vel<<endl;
        ros::spinOnce();
        loop_rate.sleep();
    }

}

void Lane_Following::subs_callback(const sensor_msgs::CompressedImageConstPtr& msg) {

    Mat dst;
    //frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    //gray = cv_bridge::toCvShare(msg, "mono8")->image;
    frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    Mat image,gray,binary,left_gray,right_gray;
    
    cv::warpPerspective(frame,image,Lane_Following::Homography(),frame.size());
    cv::cvtColor(image,gray,CV_BGRA2GRAY);
    cv::imshow("window",gray);
    for(int i=0;i<gray.rows;i++){
        auto* data = gray.ptr<uchar>(i);
        for(int j=0;j<gray.cols;j++){
            if(data[j]<10 || i>130){
                data[j]=255;
            }
        }
    }

    threshold(gray, binary, 70, 255, THRESH_BINARY);
    for(int i=0;i<binary.rows;i++){
        auto* data = binary.ptr<uchar>(i);
        for(int j=0;j<binary.cols;j++){
            data[j]=255-data[j];
        }
    }
    cv::imshow("binary",binary);

    //dst = gray(Rect(30, gray.rows * 2/3, 290, gray.rows *1/ 3));

    vector< vector<Point> > Contours;
    vector< vector<Point> > Contours_inarea;
    vector< vector<Point> > Contours_sorted;
    vector< vector<Point> > Contours_left;
    vector< vector<Point> > Contours_right;
    vector<Vec4i> hierarchy;
    cv::findContours(binary,Contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE,Point());
    
    vector<double> Contour_Areas;
    cout<<" Contours.size() = "<<Contours.size()<<endl;

    for (int i = 0; i < Contours.size(); i++)
    {
        double Area=contourArea(Contours[i]);
        if(Area>30/*|| Area>200*/){
            Contours_inarea.push_back(Contours[i]);
            Contour_Areas.push_back(Area);
        }
    }
   
    for (int i = 0; i < Contours_inarea.size(); i++)
    {
        for(int j=0;j<Contours_inarea.size();j++){
            if( Contour_Areas[j]>Contour_Areas[i] ){
                Contour_Areas[i]+=Contour_Areas[j];
                Contour_Areas[j]=Contour_Areas[i]-Contour_Areas[j];
                Contour_Areas[i]-=Contour_Areas[j];
                
                vector<Point> temp=Contours_inarea[i];
                Contours_inarea[i]=Contours_inarea[j];
                Contours_inarea[j]=temp;       
            }
        }
    }
    
    for (int i = 0; i < Contours_inarea.size(); i++){
        if( i<2 ){
            Contours_sorted.push_back(Contours_inarea[i]);
        }
    }

    
    Mat left_image=Mat::zeros(image.size(),CV_8UC3);
    Mat right_image=Mat::zeros(image.size(),CV_8UC3);
    cv::Moments M1;
    cv::Moments M2;
    cv::drawContours(image,Contours_sorted,-1,Scalar(0,0,255),1);
    imshow("src",frame);
    imshow("warp",image);
    cout<<" Contours.size() = "<<Contours_inarea.size()<<endl;



    if(Contours_sorted.size()>0){
        Contours_left.push_back(Contours_sorted[0]);
        cv::drawContours(left_image,Contours_left,-1,Scalar(255,0,0),1);
        M1 = cv::moments(Contours_left[0]);

        if( ( !Contours_sorted[1].empty() ) && Contours_sorted.size()>=2  ){
            Contours_right.push_back(Contours_sorted[1]);
            cv::drawContours(right_image,Contours_right,-1,Scalar(0,0,255),1);
            M2 = cv::moments(Contours_right[0]);
        }



        int cx1,cx2,cy1,cy2;
     
    
    
   

    
        if(M1.m00>0){
            cx1 = int(M1.m10/M1.m00);
            cy1 = int(M1.m01/M1.m00);

        }
        if(M2.m00>0){
            cx2 = int(M2.m10/M2.m00);
            cy2 = int(M2.m01/M2.m00);
        }
        if(cx2<cx1){
                cx1+=cx2;
                cx2=cx1-cx2;
                cx1-=cx2;
                cy1+=cy2;
                cy2=cy1-cy2;
                cy1-=cy2;
        }   
        cout<<cx1<<" "<<cx2<<endl;
        int fpt_x = (cx1 + cx2)/2;
        int fpt_y = (cy1 + cy2)/2 + 2*image.rows/3;

        cv::circle(image, cv::Point(cx1, cy1), 10, (0,0,255), -1);
        cv::circle(image, cv::Point(cx2, cy2), 10, (0,255,0), -1);
        cv::circle(image, cv::Point(fpt_x, fpt_y), 10, (0,0,0), -1);
        error2 = 0.3*(image.cols/2 - fpt_x);
    }
       
    
    
    
    
    
    imshow("left_image",left_image);
    imshow("right_image",right_image);
    // cv::cvtColor(left_image,left_gray,CV_BGRA2GRAY);
    // cv::cvtColor(right_image,right_gray,CV_BGRA2GRAY);
    // threshold(left_gray,left_gray,90, 255, THRESH_BINARY);
    // threshold(right_gray,right_gray,90, 255, THRESH_BINARY);

    
    // imshow("right_image",right_gray);
    /*Mat yellow_labels, yellow_stats, yellow_centroids;
    Mat white_labels, white_stats, white_centroids;*/
    

    Contours.clear();
    Contours_inarea.clear();
    Contours_sorted.clear();
    Contours_left.clear();
    Contours_right.clear();
	
    waitKey(1);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "lanefollowing");
    Lane_Following lf;
    return 0;
}
