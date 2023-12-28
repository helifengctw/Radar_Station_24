#include <cstdio>
#include <iostream>


void get_string(std::string *ss){
    std::string temp = "jjj";
    *ss = temp;
}

int main(int argc, char ** argv)
{
    (void) argc;
    (void) argv;

    printf("hello world yolov5_detect package\n");
    std::string s1 = "aaa";
    std::cout << s1 << std::endl;
    get_string(&s1);
    std::cout << s1 << std::endl;
    return 0;
}
