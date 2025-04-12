#ifndef SETTING_H
#define SETTING_H

#include <string>
#include <iostream>
#include <fstream>
#include <thread>
#include<opencv2/core/core.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

namespace OCRSLAM {

// class mapText;
// class mapPts;
// class keyframe;

typedef Eigen::Matrix<double,3,3> Mat33;
typedef Eigen::Matrix<double,4,4> Mat44;
typedef Eigen::Matrix<double,3,1> Mat31;
typedef Eigen::Matrix<double,4,1> Mat41;
typedef Eigen::Matrix<double,1,4> Mat14;
typedef Eigen::Matrix<double,1,3> Mat13;
typedef Eigen::Matrix<double,1,1> Mat11;

typedef Eigen::Matrix<double,3,1> Vec3;
typedef Eigen::Matrix<double,2,1> Vec2;

typedef pair<int,int> match;
typedef pair<int,double> matchRes;
// typedef pair<keyframe*,int> CovKF;  // each keyframe and its covisible points number

// semantic ----------
struct TextInfo
{
    string mean;            // the recognition meaning
    double score;           // recognition res score
    double score_semantic;  // S_semantic = S_geo+S_mean (smaller better)
    // int lang;               // language: english -- 0; Chinese -- 1; Chinese+english -- 2
};

struct MatchmapTextRes
{
    mapText* mapObj;
    double dist;
    double score;
};

struct Spot {
    std::vector<Eigen::Vector3d> corners; // 使用cv::Point2d存储双精度浮点坐标
    int vacant; //1为占用
};



class setting
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    setting();
}

}

#endif // SETTING_H
