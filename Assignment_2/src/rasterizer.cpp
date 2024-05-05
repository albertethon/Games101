// clang-format off
//
// Created by goksu on 4/6/19.
//
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/Core/GlobalFunctions.h>
#include <vector>
#include "rasterizer.hpp"
#include <opencv4/opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

// check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    Vector3f v_0p(x-_v[0][0],y-_v[0][1],0.0f);
    Vector3f v_01(_v[1][0] - _v[0][0],_v[1][1] - _v[0][1],0.0f);
    Vector3f v_1p(x-_v[1][0],y-_v[1][1],0.0f);
    Vector3f v_12(_v[2][0] - _v[1][0],_v[2][1] - _v[1][1],0.0f);
    Vector3f v_2p(x-_v[2][0],y-_v[2][1],0.0f);
    Vector3f v_20(_v[0][0] - _v[2][0],_v[0][1] - _v[2][1],0.0f);

    Vector3f c1 = v_01.transpose().cross(v_0p);
    Vector3f c2 = v_12.transpose().cross(v_1p);
    Vector3f c3 = v_20.transpose().cross(v_2p);
    
    return (c1.transpose()*c2 > 0 
            && c1.transpose()*c3 > 0
            && c2.transpose()*c3 > 0);
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

int rst::rasterizer::_get_sub_sample_id(int x,int y,int k){
    return (height - 1- y) * width * 4 + (width - 1 - x) * 4 + k;
}

Eigen::Vector3f rst::rasterizer::_get_sample_color(int x,int y){
    Eigen::Vector3f sum{0.0f,0.0f,0.0f};
    auto index = _get_sub_sample_id(x, y, 0);
    for(int i=0;i<4;++i){
        sum += subsample_color_buf[index+i];
    }
    return sum/4.0f;
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    // Find out the bounding box of current triangle.
    auto v = t.toVector4();
    int l=INT32_MAX;
    int r=INT32_MIN;
    int top=INT32_MIN;
    int bottom=INT32_MAX;

    l       = floor(std::min(v[0].x(),std::min(v[1].x(),v[2].x())));
    r       = ceil(std::max(v[0].x(),std::max(v[1].x(),v[2].x())));
    bottom  = floor(std::min(v[0].y(),std::min(v[1].y(),v[2].y())));
    top     = ceil(std::max(v[0].y(),std::max(v[1].y(),v[2].y())));

    // iterate through the pixel and find if the current pixel is inside the triangle
    for(int i=l;i<=r;++i){
        for(int j=bottom;j<=top;++j){
            // If so, use the following code to get the interpolated z value.
            bool check_hit=false;
            // MASS
            for(int k=0;k<4;++k){
                if(insideTriangle(i+step[2*k],j+step[2*k+1], t.v)){
                    auto[alpha, beta, gamma] = computeBarycentric2D(i+step[2*k], j+step[2*k+1], t.v);
                    float w_reciprocal = 1.0f/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;

                    int _id = _get_sub_sample_id(i, j, k);
                    
                    if(subsample_depth_buf[_id] > z_interpolated){
                        subsample_depth_buf[_id] = z_interpolated;
                        subsample_color_buf[_id] = t.getColor();
                        check_hit = true;
                    }
                }
            }
            if(check_hit){
                set_pixel(Vector3f(i,j,0), _get_sample_color(i, j));
            }
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(subsample_depth_buf.begin(),subsample_depth_buf.end(),
        std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    subsample_depth_buf.resize(w*h*4);
    subsample_color_buf.resize(w*h*4);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on