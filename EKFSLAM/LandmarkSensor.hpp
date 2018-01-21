//
//  LandmarkSensor.hpp
//  EKFSLAM
//
//  Created by HAN on 1/20/18.
//  Copyright © 2018 Zhijun Han. All rights reserved.
//

#ifndef LandmarkSensor_hpp
#define LandmarkSensor_hpp

#include "GeneralHeader.h"

class LandmarkSensor {
    /*
     Class to simulate a landmark sensor
     This is a basic range-bearing sensor in 2D
     */
    
public:
    
    LandmarkSensor();
    
    LandmarkSensor(MatrixXf & Y, MatrixXf & R, float alpha):Y(Y), R(R), alpha(alpha) {}
    
    /*
     Destructor.
     */
    //~LandmarkSensor();
    
    MatrixXf simulate(RowVectorXf &);
    
private:
    /*
     member variables:
     Y - matrix (m,2) locations of the landmarks
     R - noise covariance associated with range and bearing
     alpha - FOV of the camera in radians
     */
    MatrixXf Y, R;
    float alpha;
    
};

#endif /* LandmarkSensor_hpp */
