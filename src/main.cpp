#include <opencv2/opencv.hpp>
#include "../function/color_recognition.hpp"
#include "../function/PID.hpp"
#include <yaml-cpp/yaml.h>
#include <unistd.h>
#include <iostream>

void read_PID(const YAML::Node &pid_yaml, PIDController &pid)
{
    pid.Kd = pid_yaml["Kd"].as<double>();
    pid.Ki = pid_yaml["Ki"].as<double>();
    pid.Kp = pid_yaml["Kp"].as<double>();
    pid.limMax = pid_yaml["limMax"].as<double>();
    pid.limMin = pid_yaml["limMin"].as<double>();
    pid.limMaxInt = pid_yaml["limMaxInt"].as<double>();
    pid.limMinInt = pid_yaml["limMinInt"].as<double>();
    pid.T = pid_yaml["T"].as<double>();
    pid.tau = pid_yaml["tau"].as<double>();
}

int main()
{
    YAML::Node pid_yaml = YAML::LoadFile("../config/msg.yaml");
    PIDController pid_x, pid_y;
    PIDController_Init(pid_x);
    PIDController_Init(pid_y);
    read_PID(pid_yaml, pid_x);
    read_PID(pid_yaml, pid_y);
    print_PID(pid_x);

    cv::VideoCapture cap;
    cv::Mat img, img_two, result_img;
    cv::Point2f photo_center, prephoto_center;
    cap.open(2, cv::CAP_V4L2);
    if (cap.isOpened())
    {
        cv::namedWindow("Video");
    }
    else
    {
        std::cout << "failed" << std::endl;
        return 1;
    }
    cap >> img;
    std::cout <<"img_x="<<img.cols<<"\timg_y="<<img.rows<<std::endl;
    std::cout << "vel_x\t"
              << "vel_y" << std::endl;
     while (1)
    {
        prephoto_center = photo_center;
        int key_value = cv::waitKey(1);
        if (key_value == 27)
        {
            break;
        }
        cap >> img;
        result_img = img.clone();
        color::color_Range(img, img_two, color::red);
        bool appear = color::color_center(img_two, result_img, photo_center);
        imshow("Video", result_img);
        if (appear)
        {
            PIDController_Update(pid_x, photo_center.x, 320, pid_yaml["coff"].as<double>());
            PIDController_Update(pid_y, photo_center.y, 240, pid_yaml["coff"].as<double>());
        }
        else
        {
            pid_x.out = 0;
            pid_y.out = 0;
        }
        std::cout << pid_x.out << "\t" << pid_y.out << std::endl;
        sleep(0.04);
    }
    cv::destroyAllWindows(); //破坏窗口
    cap.release();           //释放内存
    return 0;
}
