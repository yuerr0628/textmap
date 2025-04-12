#ifndef P3P_H
#define P3P_H

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <GeographicLib/UTMUPS.hpp>
using namespace std;
// #include <message_filters/sync_policies/ApproximateTime.h>
// #include "license_plate_recognition/PlateRecognition.h" 

// 类型定义
typedef Eigen::Vector3d Point3D;
typedef Eigen::Vector2d Point2D;
typedef Eigen::Matrix3d Matrix3x3;
typedef Eigen::Vector3d Vector3;


// 车牌信息，包括四个角点和车牌号
struct LicensePlate {
    std::vector<Point3D> points;  // 四个角点
    std::string plateNumber;      // 车牌号码
    double confidence;
    double timestamp;
};


// 相机内参矩阵（假设是给定的）
class P3P{
public:
 
// extern Matrix3x3 K;

// 归一化2D点
// std::vector<Point2D> normalizePoints(const std::vector<Point2D>& pts2D, const Matrix3x3& K);

// P3P算法：通过3个3D点和2D点计算相机位姿
void P3PComputePoses(const std::vector<Point3D>& pts3D, const std::vector<Point2D>& pts2D,std::vector<Point3D>& camera_coords );
int solve_for_lengths(double lengths[4][3], double distances[3], double cosines[3]);
int SolveQuartic(const Eigen::Matrix<double, 5, 1>& factors, Eigen::Vector4d* real_roots);
bool align(double M_end[3][3],
                double X0, double Y0, double Z0,
                double X1, double Y1, double Z1,
                double X2, double Y2, double Z2,
                double R[3][3], double T[3]);
bool jacobi_4x4(double * A, double * D, double * U);


// 优化步骤：使用第四个点来调整相机位姿
// void optimizePoseWith4thPoint(const Matrix3x3& R1, const Vector3& T1, 
                            //   const Matrix3x3& R2, const Vector3& T2,
                            //   const std::vector<Point3D>& pts3D, const std::vector<Point2D>& pts2D,
                            //   Matrix3x3& R, Vector3& T);



};

class Camera
{
public:
    Eigen::Matrix2d inv_stretch_mat; // Inverse stretching matrix
    Eigen::Vector2d img_center;       // Image center
    Eigen::VectorXd pol;               // Polynomial coefficients
    int length_pol;                    // Length of polynomial

      Camera(Eigen::Matrix2d stretchMatrix, Eigen::Vector2d center, Eigen::VectorXd polynomial)
    {
        inv_stretch_mat = stretchMatrix.inverse();
        img_center = center;
        pol = polynomial;
        length_pol = polynomial.size();
    }

    Eigen::Vector3d normalizePoints(const Eigen::Vector2d& points2D, double points_depth);


};

#endif // P3P_H
