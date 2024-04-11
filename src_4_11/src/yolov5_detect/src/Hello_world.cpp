#include <cstdio>
#include <iostream>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

void get_string(string *ss){
    string temp = "jjj";
    *ss = temp;
}

cv::Mat get_mat(){
    cv::Mat ta_src = cv::Mat::ones(320, 320, CV_8UC3);
    cv::imshow("t1", ta_src);
    cv::waitKey(0);
    return ta_src;
}

void hhh(cv::Mat mm) {
    cv::Point2f p;
    p.x = 200;
    p.y = 200;
    cv::circle(mm, p, 5, cv::Scalar(0xFF, 0xFF, 0xFF), -1);
}


int main(int argc, char ** argv)
{
    Mat src_background = imread("/home/hlf/Downloads/myFiles/test/close/1.jpg", 0);
    Mat src_1 = imread("/home/hlf/Downloads/myFiles/test/close/57.jpg", 0);
    Mat img_diff;
    absdiff(src_background, src_1, img_diff);
    imshow("hhh", img_diff);
    waitKey(0);




//    cv::Mat close_depthes = cv::Mat::zeros(512, 512, CV_32FC1);
//    hhh(close_depthes);
//    cv::imshow("aaa", close_depthes);
//    cv::waitKey(0);

//    Mat far_T = Mat::ones(3, 1, CV_64FC1);
//    far_T.at<double>(2) = 3;
//    cout  << "hjhhhh       " << far_T.at<double>(2) << endl;
//
//    cv::Mat far_CamMatrix_ = Mat::zeros(3, 3, CV_64FC1);
//    far_CamMatrix_.at<double>(2, 2) = 8;
//    cout << "sassa    " << far_CamMatrix_.at<double>(2, 2) << endl ;
}
