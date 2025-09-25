#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

int main(int, char **)
{
    cv::Mat imhsv,imhsv1,gray,gray1,dst,med1,ges1,dst1;
    cv::Mat src1 = cv::imread("../../resources/test_image.png");
    cv::Mat src1_2 = src1.clone();
    cv::Mat kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    cv::cvtColor(src1, gray1, cv::COLOR_BGR2GRAY);//灰度图
    cv::cvtColor(src1, imhsv1, cv::COLOR_BGR2HSV);//hsv图
    cv::medianBlur(src1, med1, 3);//中值滤波
    cv::GaussianBlur(src1, ges1, cv::Size (3,3), 1.5);//高斯滤波
    
    cv::Mat mask_red;
    std::vector<std::vector<cv::Point>> contours;
    double s = 0;
    cv::inRange(imhsv1, cv::Scalar(0,100,100), cv::Scalar(10,255,255), mask_red);//提取红色部分
    //cv::dilate(mask_red, mask_red, kernel);
    cv::findContours(mask_red,contours,cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (size_t i = 0; i < contours.size(); ++i) {
        s += cv::contourArea(contours[i]);
        cv::Rect bounding_box = cv::boundingRect(contours[i]);
        cv::rectangle(src1, bounding_box, cv::Scalar(255, 0, 0), 2); // 用蓝色框标记红色bounding_box
    }
    cv::drawContours(src1, contours, -1, cv::Scalar(0, 255, 0), 2);//用绿色绘制红色外轮廓
    cv::imshow("src1",src1);
    std::cout<<"轮廓面积: "<<s<<std::endl;//输出红色面积

    cv::threshold(gray1, dst1, 175, 255, cv::THRESH_BINARY);//二值化
    cv::dilate(dst1, dst1, kernel);//膨胀
    cv::erode(dst1, dst1, kernel);//腐蚀


    // 创建一个mask来避免填充已经访问过的区域
    cv::Mat mask = cv::Mat::zeros(dst1.rows +2,dst1.cols +2, CV_8UC1);

    // 使用OpenCV的floodFill函数进行漫水填充
    cv::floodFill(dst1, mask, cv::Point(0,0), cv::Scalar(0, 0, 255), nullptr, cv::Scalar(10, 10, 10), cv::Scalar(10, 10, 10));
    cv::imshow("flood",dst1);

    //图像绘制
    cv::Mat img(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::circle(img, cv::Point(250, 250), 30, cv::Scalar(0,0,255), 2 );
    cv::rectangle(img, cv::Point(200, 200), cv::Point(300, 300), cv::Scalar(255,0,0), 3);
    cv::putText(img, "text", cv::Point(220, 250), cv::FONT_HERSHEY_COMPLEX, 1.0, cv::Scalar(0,255,0), 2);
    cv::imshow("pic",img);

    //图像操作
    cv::Point2f center(src1_2.cols / 2.0F, src1_2.rows / 2.0F); // 旋转中心
    cv::Mat rotMat = cv::getRotationMatrix2D(center, 35, 1.0); // 35度，缩放因子1.0

    cv::Mat rotated;
    cv::warpAffine(src1_2, rotated, rotMat, src1_2.size());
    cv::Mat cropped = src1_2(cv::Rect(0, 0, src1_2.cols /2 , src1_2.rows /2));
    cv::imshow("rotated", rotated);
    cv::imshow("cutted", cropped);

    //装甲板识别部分
    cv::Mat src = cv::imread("../../resources/test_image_2.png"); 
    cv::cvtColor(src, imhsv, cv::COLOR_BGR2HSV);
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, dst, 200, 255, cv::THRESH_BINARY);
    cv::dilate(dst, dst, kernel);
    cv::erode(dst, dst, kernel);
    std::vector<std::vector<cv::Point>> contours1;
    cv::findContours(dst, contours1, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<cv::Rect> rects ;
    for (size_t i = 0; i < contours1.size(); i++) {
        if (cv::contourArea(contours1[i]) > 100){
            double area = cv::contourArea(contours1[i]);
            cv::Rect rect = cv::boundingRect(contours1[i]);
            if (area / rect.area() > 0.7){
                cv::rectangle(dst, rect, cv::Scalar(255, 255, 0), 2);
                rects.push_back(rect);
            }
        }
    }

    int x1 = 2220000,x2 = 0,y1 = 2200000,y2 = 0;
    //在多灯条情况下，或可通过限制搜寻范围帮助找到正确的灯条对，在光源复杂情况下，可以通过灯条形态特征和相对关系确定灯条，也可以结合识别数字，
    //在确定灯条和装甲板几何关系的情况下可以将框选范围扩大到装甲板
     for (int i = 0 ; i <rects.size() ; ++i){
        if (rects[i].x < x1) {
            x1 = rects[i].x;
        }
        if (rects[i].y < y1) {
            y1 = rects[i].y;
        }
        if (rects[i].x + rects[i].width > x2) {
            x2 = rects[i].x + rects[i].width;
        }
        if (rects[i].y + rects[i].height > y2) {
            y2 = rects[i].y + rects[i].height;
        }
    }
    cv::Rect rect;
    rect.x = x1;
    rect.width = x2-x1;
    rect.y = y1-0.2*(y2-y1);
    rect.height = 1.4*(y2-y1);
    cv::rectangle(src, rect, cv::Scalar(255, 255, 0), 2);
    cv::namedWindow("src",cv::WINDOW_NORMAL);
    cv::imshow("src", src);
    cv::namedWindow("image_hsv1",cv::WINDOW_NORMAL);
    cv::imshow("image_hsv1", dst);
    cv::waitKey(0);
    cv::destroyAllWindows();

    //写入图片
    cv::imwrite("../../resources/imhsv.png", imhsv1);
    cv::imwrite("../../resources/gray.png", gray1);
    cv::imwrite("../../resources/medianblur_img.png", med1);
    cv::imwrite("../../resources/gaussianBlur_img.png", ges1);
    cv::imwrite("../../resources/mask_red.png", mask_red);
    cv::imwrite("../../resources/flood_fill_img.png", dst1);
    cv::imwrite("../../resources/draw_img.png", img);
    cv::imwrite("../../resources/rotated_img.png", rotated);
    cv::imwrite("../../resources/cropped_img.png", cropped);
    cv::imwrite("../../resources/detection_result.png", src);
    return 0;
}
