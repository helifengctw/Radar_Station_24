#include <cstdio>
#include <iostream>

#include <opencv2/opencv.hpp>

using namespace std;

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

int main(int argc, char ** argv)
{
    (void) argc;
    (void) argv;

    printf("hello world yolov5_detect package\n");
    string s1 = "aaa";
    cout << s1 << endl;
    get_string(&s1);
    cout << s1 << endl;
    
    for(int i = 0; i < 88; i++){
        cout << i << endl;
        if (i > 5) {
            break;
        }
    }
    int a = 909, b = 789;
    cout << a++ << endl << ++b << endl;

    cv::Mat src1 = get_mat();
    cv::imshow("test", src1);
    cv::waitKey(0);
}
