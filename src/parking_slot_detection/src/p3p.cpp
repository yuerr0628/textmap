#include "p3p.h"
#include <iostream>
#include <cmath>

// 假设相机的内参矩阵K
/// Given 3D distances between three points and cosines of 3 angles at the apex, calculates
/// the lentghs of the line segments connecting projection center (P) and the three 3D points (A, B, C).
/// Returned distances are for |PA|, |PB|, |PC| respectively.
/// Only the solution to the main branch.
/// Reference : X.S. Gao, X.-R. Hou, J. Tang, H.-F. Chang; "Complete Solution Classification for the Perspective-Three-Point Problem"
/// IEEE Trans. on PAMI, vol. 25, No. 8, August 2003
/// \param lengths3D Lengths of line segments up to four solutions.
/// \param dist3D Distance between 3D points in pairs |BC|, |AC|, |AB|.
/// \param cosines Cosine of the angles /_BPC, /_APC, /_APB.
/// \returns Number of solutions.
/// WARNING: NOT ALL THE DEGENERATE CASES ARE IMPLEMENTED

Matrix3x3 K;

Eigen::Vector3d Camera::normalizePoints(const Eigen::Vector2d& points2D, double points_depth)
    {
        Eigen::Vector3d points3D;

        // Minus the image center
        Eigen::Vector2d normalized_points2D = points2D- img_center;
        // cout<<"normalized_points2D="<<normalized_points2D<<endl;

        normalized_points2D = inv_stretch_mat * normalized_points2D;

        // Calculate norm
        double norm = normalized_points2D.norm();

        // Generate norm polynomial
        std::vector<double> norm_poly(length_pol);
        for (int i = 0; i < length_pol; ++i) {
            norm_poly[i] = std::pow(norm, i);
        }

        // zp calculation
        double zp = pol.dot(Eigen::Map<const Eigen::VectorXd>(norm_poly.data(), norm_poly.size()));
        // Calculate lambda
        double lamda = points_depth / zp;

        points3D[0] = (lamda * normalized_points2D[0]);
        points3D[1] = (lamda * normalized_points2D[1]);
        points3D[2] = points_depth;
        std::cout << "normalbefore:"<<points3D[0] << ", " << points3D[1] << ", " << points3D[2] << std::endl;

        double norm1 = sqrt(points3D[0] *points3D[0] +points3D[1] * points3D[1] + 1);
        points3D[2]=1. / norm1; points3D[0] *=points3D[2];points3D[1] *=points3D[2];
        // std::cout << "normal:"<<points3D[0] << ", " << points3D[1] << ", " << points3D[2] << std::endl;

        // Assuming r2 is already a rotation matrix
         double pi = M_PI;
        // Rotate points
       // 创建旋转矩阵r
        Eigen::AngleAxisd angleAxisX(M_PI / 2, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd angleAxisY(0, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd angleAxisZ(M_PI / 2, Eigen::Vector3d::UnitZ());
        Eigen::Matrix3d r = (angleAxisZ * angleAxisY * angleAxisX).toRotationMatrix();

        points3D = r * points3D;
        std::cout << points3D[0] << ", " << points3D[1] << ", " << points3D[2] << std::endl;
        // cout<<"outnormal"<<endl;
        return points3D;
    }

// P3P算法，给定3个3D点和3个2D点，计算旋转矩阵R和位移向量T
void P3P::P3PComputePoses(const std::vector<Point3D>& worldpts3D, const std::vector<Point2D>& pts2D,std::vector<Point3D>& camera_coords ) {
    
    // 归一化2D点
    // double c = 1.001743, d = -0.003094, e = 0.000380;
    double c = 1.001151, d = -0.004470, e = 0.002788;

    // Stretching matrix (computed from affine parameters)
    Eigen::Matrix2d stretchMatrix;
    stretchMatrix << c, d,
                     e, 1;

    // Image center
    Eigen::Vector2d center(545.665896, 948.540793);

    // Polynomial coefficients for direct mapping function
    Eigen::VectorXd polynomial(5); // Degree 5 polynomial
    // polynomial << -4.706897e+02, 0.000000e+00 ,7.738225e-04, -5.910803e-07, 6.981927e-10 ;
    polynomial << -4.690665e+02, 0.000000e+00 ,6.905341e-04, -3.800770e-07, 5.395195e-10 ;
    // Initialize Camera object
    Camera myCamera(stretchMatrix, center, polynomial);
    std::vector<Point3D> pts3D_normalized;
    double depth=1;
    for(int i=0; i<pts2D.size(); i++)
    {
        // cout<<"pts2D"<<pts2D.size()<<pts2D[i]<<endl;
        Eigen::Vector3d points_c=myCamera.normalizePoints(pts2D[i], depth);
        pts3D_normalized.push_back(points_c);
    }
    
    double mu0=pts3D_normalized[0][0],mv0=pts3D_normalized[0][1],mk0=pts3D_normalized[0][2];
    double mu1=pts3D_normalized[1][0],mv1=pts3D_normalized[1][1],mk1=pts3D_normalized[1][2];
    double mu2=pts3D_normalized[2][0],mv2=pts3D_normalized[2][1],mk2=pts3D_normalized[2][2];
    double mu3=pts3D_normalized[3][0],mv3=pts3D_normalized[3][1],mk3=pts3D_normalized[3][2];
    // // 计算3D点之间的相对位置（向量差）
    Eigen::Vector3d p1_3D = worldpts3D[0];
    Eigen::Vector3d p2_3D = worldpts3D[1];
    Eigen::Vector3d p3_3D = worldpts3D[2];
    Eigen::Vector3d p4_3D = worldpts3D[3];
    double  X3=p4_3D[0],Y3=p4_3D[1],Z3=p4_3D[2];

    // Calculate distances
    double distances[3] = {
        (worldpts3D[1] - worldpts3D[2]).norm(),
        (worldpts3D[0] - worldpts3D[2]).norm(),
        (worldpts3D[0] - worldpts3D[1]).norm()
    };

    // if (d21.cross(d31).squaredNorm() == 0.0)
    //     return -1;
    // Calculate cosines//计算角度cos
    // double cosines[3] = {
    //     mu[1] * mu[2] + mv[1] * mv[2] + mk[1] * mk[2],
    //     mu[0] * mu[2] + mv[0] * mv[2] + mk[0] * mk[2],
    //     mu[0] * mu[1] + mv[0] * mv[1] + mk[0] * mk[1]
    // };
        double cosines[3] = {
        mu1 * mu2 + mv1 * mv2 + mk1 * mk2,
        mu0 * mu2 + mv0 * mv2 + mk0 * mk2,
        mu0 * mu1 + mv0 * mv1 + mk0 * mk1
    };
    cout<<"cosine="<<cosines[0]<<","<<cosines[1]<<","<<cosines[2]<<endl;


        //吴消元法求解PA，PB，PC的值，有四组解；
    double lengths[4][3];
    int n = solve_for_lengths(lengths, distances, cosines);
    cout<<"N="<<n<<endl;
    


 
    int nb_solutions = 0;
    // cout<<"outlength1"<<endl;
    // double reproj_errors[4];
    // cout<<"outlength2"<<endl;
    double R[4][3][3]; double t[4][3];
      int ns = 0;
    double min_reproj = 0;
    Eigen::Vector3d point13D;
    Eigen::Vector3d point23D;
    Eigen::Vector3d point33D;
    Eigen::Vector3d point43D;
    // cout<<"outlength3"<<endl;
    for(int i = 0; i < n; i++) {
        // cout<<"outlength4"<<endl;
        double M_orig[3][3];
 
        //对每个点求坐标值，单位向量乘以距离；
        M_orig[0][0] = lengths[i][0] * mu0;
        M_orig[0][1] = lengths[i][0] * mv0;
        M_orig[0][2] = lengths[i][0] * mk0;
        
 
        M_orig[1][0] = lengths[i][1] * mu1;
        M_orig[1][1] = lengths[i][1] * mv1;
        M_orig[1][2] = lengths[i][1] * mk1;
 
        M_orig[2][0] = lengths[i][2] * mu2;
        M_orig[2][1] = lengths[i][2] * mv2;
        M_orig[2][2] = lengths[i][2] * mk2;
        cout<<lengths[i][0]<<","<<lengths[i][1]<<","<<lengths[i][2]<<endl;
        
        //计算每个解对应的位姿矩阵R，t
        if (!align(M_orig, p1_3D[0],p1_3D[1], p1_3D[2],  p2_3D[0],p2_3D[1], p2_3D[2],  p3_3D[0],p3_3D[1], p3_3D[2], R[nb_solutions], t[nb_solutions]))
            continue;

                //利用第4个点辅助选择
        // if (p4p)
        // {
            double X3p = R[nb_solutions][0][0] * X3 + R[nb_solutions][0][1] * Y3 + R[nb_solutions][0][2] * Z3 + t[nb_solutions][0];
            double Y3p = R[nb_solutions][1][0] * X3 + R[nb_solutions][1][1] * Y3 + R[nb_solutions][1][2] * Z3 + t[nb_solutions][1];
            double Z3p = R[nb_solutions][2][0] * X3 + R[nb_solutions][2][1] * Y3 + R[nb_solutions][2][2] * Z3 + t[nb_solutions][2];
            double mu3p = X3p / Z3p;
            double mv3p = Y3p / Z3p;
            double reproj_errors=(mu3p - mu3) * (mu3p - mu3) + (mv3p - mv3) * (mv3p - mv3);
            if (i == 0 || min_reproj > reproj_errors) {
                ns = i;
                min_reproj = reproj_errors;
                point13D[0]=M_orig[0][0];point13D[1]=M_orig[0][1];point13D[2]=M_orig[0][2];
                point23D[0]=M_orig[1][0];point23D[1]=M_orig[1][1];point23D[2]=M_orig[1][2];
                point33D[0]=M_orig[2][0];point33D[1]=M_orig[2][1];point33D[2]=M_orig[2][2];
                point43D[0]=X3p;
                point43D[1]=Y3p;
                point43D[2]=Z3p;
            }
            // reproj_errors[nb_solutions] = (mu3p - mu3) * (mu3p - mu3) + (mv3p - mv3) * (mv3p - mv3);
        // }
 
        nb_solutions++;
    }
    camera_coords.push_back(point13D);
    camera_coords.push_back(point23D);
    camera_coords.push_back(point33D);
    camera_coords.push_back(point43D);
     
    //   if (p4p)
    // {
        //sort the solutions
    //     for (int i = 1; i < nb_solutions; i++)
    //     {
    //         cout<<"insort"<<endl;
    //         for (int j = i; j > 0 && reproj_errors[j-1] > reproj_errors[j]; j--)
    //         {
    //             std::swap(reproj_errors[j], reproj_errors[j-1]);
    //             std::swap(R[j], R[j-1]);
    //             std::swap(t[j], t[j-1]);
    //         }
    //     }
    // // }

        

 
}


int P3P::solve_for_lengths(double lengths[4][3], double distances[3], double cosines[3])
{
    // cout<<"length"<<endl;
    //吴消元法，数据准备
    double p = cosines[0] * 2;
    double q = cosines[1] * 2;
    double r = cosines[2] * 2;

    double inv_d22 = 1. / (distances[2] * distances[2]);
    double a = inv_d22 * (distances[0] * distances[0]); // w
    double b = inv_d22 * (distances[1] * distances[1]); //v

    double a2 = a * a, b2 = b * b, p2 = p * p, q2 = q * q, r2 = r * r;
    double pr = p * r, pqr = q * pr;
    // cout<<"p2 + q2 + r2 - pqr - 1="<<p2 + q2 + r2 - pqr - 1<<endl;

    // Check reality condition (the four points should not be coplanar)
    if (p2 + q2 + r2 - pqr - 1 == 0)
        return 0;

    double ab = a * b, a_2 = 2*a;
      Eigen::Matrix<double, 5, 1> factors;

    double A = -2 * b + b2 + a2 + 1 + ab*(2 - r2) - a_2;
    factors[0]=A;

    //A, B, C, D, E 为四次多项式的系数；
    // Check reality condition
    // cout<<"A="<<A<<endl;
    if (A == 0) return 0;

    double a_4 = 4*a;

    double B = q*(-2*(ab + a2 + 1 - b) + r2*ab + a_4) + pr*(b - b2 + ab);
    double C = q2 + b2*(r2 + p2 - 2) - b*(p2 + pqr) - ab*(r2 + pqr) + (a2 - a_2)*(2 + q2) + 2;
    double D = pr*(ab-b2+b) + q*((p2-2)*b + 2 * (ab - a2) + a_4 - 2);
    double E = 1 + 2*(b - a - ab) + b2 - b*p2 + a2;
    factors[1]=B;
    factors[2]=C;
    factors[3]=D;
    factors[4]=E;

    double temp = (p2*(a-1+b) + r2*(a-1-b) + pqr - a*pqr);
    double b0 = b * temp * temp;
    // cout<<"b="<<b0<<endl;
    // Check reality condition
    if (b0 == 0)
        return 0;

    //求解四次多项式；
    Eigen::Vector4d real_roots;
  
  

    int n = SolveQuartic(factors, &real_roots);
    cout<<"root"<<real_roots<<endl;
    cout<<"solveq="<<n<<endl;

    if (n == 0)
        return 0;

    int nb_solutions = 0;
    double r3 = r2*r, pr2 = p*r2, r3q = r3 * q;
    double inv_b0 = 1. / b0;

    // For each solution of x
    for(int i = 0; i < n; i++) {
        double x = real_roots[i];

        // Check reality condition
        if (x <= 0)
            continue;

        double x2 = x*x;
        double b1 =
            ((1-a-b)*x2 + (q*a-q)*x + 1 - a + b) *
            (((r3*(a2 + ab*(2 - r2) - a_2 + b2 - 2*b + 1)) * x +

            (r3q*(2*(b-a2) + a_4 + ab*(r2 - 2) - 2) + pr2*(1 + a2 + 2*(ab-a-b) + r2*(b - b2) + b2))) * x2 +

            (r3*(q2*(1-2*a+a2) + r2*(b2-ab) - a_4 + 2*(a2 - b2) + 2) + r*p2*(b2 + 2*(ab - b - a) + 1 + a2) + pr2*q*(a_4 + 2*(b - ab - a2) - 2 - r2*b)) * x +

            2*r3q*(a_2 - b - a2 + ab - 1) + pr2*(q2 - a_4 + 2*(a2 - b2) + r2*b + q2*(a2 - a_2) + 2) +
            p2*(p*(2*(ab - a - b) + a2 + b2 + 1) + 2*q*r*(b + a_2 - a2 - ab - 1)));

        // Check reality condition
        cout<<"b1="<<b1<<endl;
        if (b1 <= 0)
            continue;

        double y = inv_b0 * b1;
        cout<<"y="<<y<<endl;
        double v = x2 + y*y - x*y*r;
         cout<<"v="<<v<<endl;
        if (v <= 0)
            continue;

        double Z = distances[2] / sqrt(v);
        double X = x * Z;
        double Y = y * Z;

        lengths[nb_solutions][0] = X;
        lengths[nb_solutions][1] = Y;
        lengths[nb_solutions][2] = Z;

        nb_solutions++;
    }

    return nb_solutions;
}



int P3P::SolveQuartic(const Eigen::Matrix<double, 5, 1>& factors, Eigen::Vector4d* real_roots) {
    double A = factors[0];
    double B = factors[1];
    double C = factors[2];
    double D = factors[3];
    double E = factors[4];

    if (A == 0) {
        // Handle cubic case if A is zero
        return -1; // Indicating input error, not a quartic equation.
    }

    double A_pw2 = A * A;
    double B_pw2 = B * B;
    double A_pw3 = A_pw2 * A;
    double B_pw3 = B_pw2 * B;
    double A_pw4 = A_pw3 * A;
    double B_pw4 = B_pw3 * B;

    double alpha = -3.0 * B_pw2 / (8.0 * A_pw2) + C / A;
    double beta = B_pw3 / (8.0 * A_pw3) - B * C / (2.0 * A_pw2) + D / A;
    double gamma = -3.0 * B_pw4 / (256.0 * A_pw4) + B_pw2 * C / (16.0 * A_pw3) - B * D / (4.0 * A_pw2) + E / A;

    double alpha_pw2 = alpha * alpha;
    double alpha_pw3 = alpha_pw2 * alpha;

    std::complex<double> P(-alpha_pw2 / 12.0 - gamma, 0.0);
    std::complex<double> Q(-alpha_pw3 / 108.0 + alpha * gamma / 3.0 - beta * beta / 8.0, 0.0);
    std::complex<double> R = -Q / 2.0 + sqrt(pow(Q, 2.0) / 4.0 + P * P * P / 27.0);

    std::complex<double> U = pow(R, (1.0 / 3.0));
    std::complex<double> y;

    if (U.real() == 0.0)
        y = -5.0 * alpha / 6.0 - pow(Q, (1.0 / 3.0));
    else
        y = -5.0 * alpha / 6.0 - P / (3.0 * U) + U;

    std::complex<double> w = sqrt(alpha + 2.0 * y);

     std::complex<double> temp;

    temp = -B / (4.0 * A)
        + 0.5 * (w + sqrt(-(3.0 * alpha + 2.0 * y + 2.0 * beta / w)));
    (*real_roots)[0] = temp.real();
    temp = -B / (4.0 * A)
        + 0.5 * (w - sqrt(-(3.0 * alpha + 2.0 * y + 2.0 * beta / w)));
    (*real_roots)[1] = temp.real();
    temp = -B / (4.0 * A)
        + 0.5 * (-w + sqrt(-(3.0 * alpha + 2.0 * y - 2.0 * beta / w)));
    (*real_roots)[2] = temp.real();
    temp = -B / (4.0 * A)
        + 0.5 * (-w - sqrt(-(3.0 * alpha + 2.0 * y - 2.0 * beta / w)));
    (*real_roots)[3] = temp.real();

  

    return 4;
}

bool P3P::align(double M_end[3][3],
                double X0, double Y0, double Z0,
                double X1, double Y1, double Z1,
                double X2, double Y2, double Z2,
                double R[3][3], double T[3])
{
    // cout<<"align"<<endl;
    // Centroids:
    double C_start[3] = {}, C_end[3] = {};
    for(int i = 0; i < 3; i++) C_end[i] = (M_end[0][i] + M_end[1][i] + M_end[2][i]) / 3;
    C_start[0] = (X0 + X1 + X2) / 3;
    C_start[1] = (Y0 + Y1 + Y2) / 3;
    C_start[2] = (Z0 + Z1 + Z2) / 3;

    // Covariance matrix s:
    double s[3 * 3] = {};
    for(int j = 0; j < 3; j++) {
        s[0 * 3 + j] = (X0 * M_end[0][j] + X1 * M_end[1][j] + X2 * M_end[2][j]) / 3 - C_end[j] * C_start[0];
        s[1 * 3 + j] = (Y0 * M_end[0][j] + Y1 * M_end[1][j] + Y2 * M_end[2][j]) / 3 - C_end[j] * C_start[1];
        s[2 * 3 + j] = (Z0 * M_end[0][j] + Z1 * M_end[1][j] + Z2 * M_end[2][j]) / 3 - C_end[j] * C_start[2];
    }

    double Qs[16] = {}, evs[4] = {}, U[16] = {};

    Qs[0 * 4 + 0] = s[0 * 3 + 0] + s[1 * 3 + 1] + s[2 * 3 + 2];
    Qs[1 * 4 + 1] = s[0 * 3 + 0] - s[1 * 3 + 1] - s[2 * 3 + 2];
    Qs[2 * 4 + 2] = s[1 * 3 + 1] - s[2 * 3 + 2] - s[0 * 3 + 0];
    Qs[3 * 4 + 3] = s[2 * 3 + 2] - s[0 * 3 + 0] - s[1 * 3 + 1];

    Qs[1 * 4 + 0] = Qs[0 * 4 + 1] = s[1 * 3 + 2] - s[2 * 3 + 1];
    Qs[2 * 4 + 0] = Qs[0 * 4 + 2] = s[2 * 3 + 0] - s[0 * 3 + 2];
    Qs[3 * 4 + 0] = Qs[0 * 4 + 3] = s[0 * 3 + 1] - s[1 * 3 + 0];
    Qs[2 * 4 + 1] = Qs[1 * 4 + 2] = s[1 * 3 + 0] + s[0 * 3 + 1];
    Qs[3 * 4 + 1] = Qs[1 * 4 + 3] = s[2 * 3 + 0] + s[0 * 3 + 2];
    Qs[3 * 4 + 2] = Qs[2 * 4 + 3] = s[2 * 3 + 1] + s[1 * 3 + 2];

    jacobi_4x4(Qs, evs, U);

    // Looking for the largest eigen value:
    int i_ev = 0;
    double ev_max = evs[i_ev];
    for(int i = 1; i < 4; i++)
        if (evs[i] > ev_max)
            ev_max = evs[i_ev = i];

    // Quaternion:
    double q[4];
    for(int i = 0; i < 4; i++)
        q[i] = U[i * 4 + i_ev];

    double q02 = q[0] * q[0], q12 = q[1] * q[1], q22 = q[2] * q[2], q32 = q[3] * q[3];
    double q0_1 = q[0] * q[1], q0_2 = q[0] * q[2], q0_3 = q[0] * q[3];
    double q1_2 = q[1] * q[2], q1_3 = q[1] * q[3];
    double q2_3 = q[2] * q[3];

    R[0][0] = q02 + q12 - q22 - q32;
    R[0][1] = 2. * (q1_2 - q0_3);
    R[0][2] = 2. * (q1_3 + q0_2);

    R[1][0] = 2. * (q1_2 + q0_3);
    R[1][1] = q02 + q22 - q12 - q32;
    R[1][2] = 2. * (q2_3 - q0_1);

    R[2][0] = 2. * (q1_3 - q0_2);
    R[2][1] = 2. * (q2_3 + q0_1);
    R[2][2] = q02 + q32 - q12 - q22;

    for(int i = 0; i < 3; i++)
        T[i] = C_end[i] - (R[i][0] * C_start[0] + R[i][1] * C_start[1] + R[i][2] * C_start[2]);

    return true;
}

bool P3P::jacobi_4x4(double * A, double * D, double * U)
{
    // cout<<"jacobi"<<endl;
    double B[4] = {}, Z[4] = {};
    double Id[16] = {1., 0., 0., 0.,
                     0., 1., 0., 0.,
                     0., 0., 1., 0.,
                     0., 0., 0., 1.};

    memcpy(U, Id, 16 * sizeof(double));

    B[0] = A[0]; B[1] = A[5]; B[2] = A[10]; B[3] = A[15];
    memcpy(D, B, 4 * sizeof(double));

    for(int iter = 0; iter < 50; iter++) {
        double sum = fabs(A[1]) + fabs(A[2]) + fabs(A[3]) + fabs(A[6]) + fabs(A[7]) + fabs(A[11]);

        if (sum == 0.0)
            return true;

        double tresh =  (iter < 3) ? 0.2 * sum / 16. : 0.0;
        for(int i = 0; i < 3; i++) {
            double * pAij = A + 5 * i + 1;
            for(int j = i + 1 ; j < 4; j++) {
                double Aij = *pAij;
                double eps_machine = 100.0 * fabs(Aij);

                if ( iter > 3 && fabs(D[i]) + eps_machine == fabs(D[i]) && fabs(D[j]) + eps_machine == fabs(D[j]) )
                    *pAij = 0.0;
                else if (fabs(Aij) > tresh) {
                    double hh = D[j] - D[i], t;
                    if (fabs(hh) + eps_machine == fabs(hh))
                        t = Aij / hh;
                    else {
                        double theta = 0.5 * hh / Aij;
                        t = 1.0 / (fabs(theta) + sqrt(1.0 + theta * theta));
                        if (theta < 0.0) t = -t;
                    }

                    hh = t * Aij;
                    Z[i] -= hh;
                    Z[j] += hh;
                    D[i] -= hh;
                    D[j] += hh;
                    *pAij = 0.0;

                    double c = 1.0 / sqrt(1 + t * t);
                    double s = t * c;
                    double tau = s / (1.0 + c);
                    for(int k = 0; k <= i - 1; k++) {
                        double g = A[k * 4 + i], h = A[k * 4 + j];
                        A[k * 4 + i] = g - s * (h + g * tau);
                        A[k * 4 + j] = h + s * (g - h * tau);
                    }
                    for(int k = i + 1; k <= j - 1; k++) {
                        double g = A[i * 4 + k], h = A[k * 4 + j];
                        A[i * 4 + k] = g - s * (h + g * tau);
                        A[k * 4 + j] = h + s * (g - h * tau);
                    }
                    for(int k = j + 1; k < 4; k++) {
                        double g = A[i * 4 + k], h = A[j * 4 + k];
                        A[i * 4 + k] = g - s * (h + g * tau);
                        A[j * 4 + k] = h + s * (g - h * tau);
                    }
                    for(int k = 0; k < 4; k++) {
                        double g = U[k * 4 + i], h = U[k * 4 + j];
                        U[k * 4 + i] = g - s * (h + g * tau);
                        U[k * 4 + j] = h + s * (g - h * tau);
                    }
                }
                pAij++;
            }
        }

        for(int i = 0; i < 4; i++) B[i] += Z[i];
        memcpy(D, B, 4 * sizeof(double));
        memset(Z, 0, 4 * sizeof(double));
    }

    return false;
}


