//
//  LandmarkSensor.cpp
//  EKFSLAM
//
//  Created by HAN on 1/20/18.
//  Copyright © 2018 Zhijun Han. All rights reserved.
//

#include "LandmarkSensor.hpp"

MatrixXf LandmarkSensor::simulate(RowVectorXf & X)
{
    /*
     inputs:
     X - matrix (T,3) of vehicle state (x, y, theta)
     
     outputs:
     Z - matrix (T,3*m) of range/bearing/signature measurements
     */
    int m = int(this->Y.rows());
    
    MatrixXf Z;
    
    for (int i = 0; i < m; ++i)
    {
        VectorXf z(3,1);
        float pos0 = pow((X(0,0) - this->Y(i,0)),2);
        float pos1 = pow((X(0,1) - this->Y(i,1)),2);
        float pos2 = pow((X(0,2) - this->Y(i,2)),2);
        
        z(0,0) = sqrt( pos0 + pos1 + pos2 );
        z(1,0) = atan2(this->Y(i,1) - X(0,1), this->Y(i,0) - X(0,0)) - X(0,2);
        
        z += this->R; // To figure out a way to insert the random multivariate Gaussian
        
        // Wrap relative bearing
        
        if (z(1,0) > PI)
            z(1,0) = z(1,0) - 2 * PI;
        else if (z(1,0) < -PI)
            z(1,0) = z(1,0) + 2 * PI;
        
        z(2,0) = this->Y(i,2);
        
        if (abs(z(1,0)) < this->alpha / 2)
            Z << z;
        
    }
    
    return Z;
}