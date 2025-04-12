#ifndef MAPTEXT_H
#define MAPTEXT_H

#include<string>
#include<thread>
#include <map>
#include<opencv2/core/core.hpp>
#include <setting.h>
// #include <tool.h>
using namespace std;

namespace OCRSLAM {

class mapText
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // apart from initial map text object state is set to GOOD at the first time.
    // all map text objects are set to IMMATURE at the 'new mapText();'
    TextStatus STATE;
    int NumObvs;
    Mat33 Covariance;   // for unmature text object theta
 
    // semantic meaning *****
    TextInfo TextMean;
    // ***************************

    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstKFrameid;

    // raw text detection of each pyramid (pyramid -> 4 features/points)
    vector<vector<Vec2>> vTextDete;
    vector<vector<Vector3d>> TextDeteV3;
    vector<mapText>text_corner;
    vector<Vec2> vTextDeteRay;  // the boxes in all pyramids have the same ray info

};

}

#endif // MAPTEXT_H
