//
// Created by LEI XU on 4/27/19.
//

#include <cmath>
#include <eigen3/Eigen/Eigen>
#include "Texture.hpp"

int Texture::rangeSafe(int x, bool isU) {

    if (x < 0)
        return 0;

    if (isU && x>= width) {
        return width - 1;
    }
    else if(!isU && x >= height) {
        return height - 1;
    }
    return x;
}

Eigen::Vector3f Texture::getColorBilinear(float u, float v){
    float u_img = u * width;
    float v_img = (1 - v) * height;
    
    int uMin = rangeSafe(floor(u_img),true);
    int uMax = rangeSafe(ceil(u_img),true);
    int vMin = rangeSafe(floor(v_img), false);
    int vMax = rangeSafe(ceil(v_img), false);
    
    auto u00 = image_data.at<cv::Vec3b>(vMax,uMin);
    auto u10 = image_data.at<cv::Vec3b>(vMax,uMax);
    auto u01 = image_data.at<cv::Vec3b>(vMin,uMin);
    auto u11 = image_data.at<cv::Vec3b>(vMin,uMax);
    
    float lerp_h = (u_img-uMin)/(uMax-uMin);
    float lerp_v = (v_img-vMin)/(vMax-vMin);

    auto u1 = (1-lerp_h)*u01 + lerp_h*u11;
    auto u0 = (1-lerp_h)*u00 + lerp_h*u10;

    auto img = (1-lerp_v)*u1 + lerp_v*u0;

    return Eigen::Vector3f(img[0],img[1],img[2]);
}
