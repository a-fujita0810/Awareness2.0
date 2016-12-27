//
//  common_math.h
//  Awareness_1.0
//
//  Created by Atsushi Fujita on 2016/12/12.
//  Copyright © 2016年 Atsushi Fujita. All rights reserved.
//

#ifndef common_math_h
#define common_math_h

#include "vehicle_detection_sensor.h"


class VectorType{
public:
    float x,y;

    VectorType(): x(0), y(0)
    {}

    VectorType(const float& initX, const float& initY): x(initX), y(initY)
    {}

    void clear() {
        x = 0;
        y = 0;
    }
}; //VectorType//


float distance2Coordinate(float a_x, float a_y, float b_x, float b_y){
	float tmp_x = a_x - b_x;
    float tmp_y = a_y - b_y;
    float result = sqrt(tmp_x * tmp_x + tmp_y * tmp_y);
	return result;
} //distance2Coordinate


//共通の関数として利用できるようにできない？
inline
VectorType rotateXY(
        const float x,
        const float y,
        const float theta)
{
    float cs = cos(theta);
    float sn = sin(theta);
    
    return VectorType(x*cs + y*sn, y*cs - x*sn);
}


inline
int crossProduct(
        const VectorType p1,
        const VectorType p2,
        const VectorType p3)
{
    float ZCoord = p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y);
    if ( ZCoord > 0 ) {return  1;} // 左
    else if ( ZCoord < 0 ) {return -1;} // 右
	else {return  0;} // 線上
}



inline
bool isDetectableBySensor(
        const VectorType relativeCoord,
        const VectorType selfVelocity)
{

    //最大検出距離50としている
    float expandDist = 50/distance2Coordinate(selfVelocity.x, selfVelocity.y, 0, 0);
    
    VectorType origin = VectorType(0.00, 0.00);
    VectorType areaCenter = VectorType(selfVelocity.x * expandDist, selfVelocity.y * expandDist);
    VectorType areaLeft = rotateXY(areaCenter.x, areaCenter.y, 3.14/3);
    VectorType areaRight = rotateXY(areaCenter.x, areaCenter.y, (-1)*3.14/3);
    VectorType areaLeftMiddle = rotateXY(areaCenter.x, areaCenter.y, 3.14/6);
    VectorType areaRightMiddle = rotateXY(areaCenter.x, areaCenter.y, (-1)*3.14/6);
    
    int pOR = crossProduct(relativeCoord, origin, areaRight);
    int pRM = crossProduct(relativeCoord, areaRight, areaRightMiddle);
    int pMC = crossProduct(relativeCoord, areaRightMiddle, areaCenter);
    int pCM = crossProduct(relativeCoord, areaCenter, areaLeftMiddle);
    int pML = crossProduct(relativeCoord, areaLeftMiddle, areaLeft);
    int pLO = crossProduct(relativeCoord, areaLeft, origin);
    
    if ((pOR > 0) && (pRM > 0) && (pMC > 0) && (pCM > 0) && (pML > 0) && (pLO > 0)) {
        return true;
    } else if ((pOR < 0) && (pRM < 0) && (pMC < 0) && (pCM < 0) && (pML < 0) && (pLO < 0)) {
        return true;
    }
    
    return false;
} //isDetectableBySensor

inline
double NormalDistribution(const double x, const double mean, const double var) {
    double coefficient = 1/sqrt(2*M_PI*var);
    double power = (-1) * (x-mean)*(x-mean) / (2*var);
    return coefficient * exp(power);
}



#endif /* common_math_h */
