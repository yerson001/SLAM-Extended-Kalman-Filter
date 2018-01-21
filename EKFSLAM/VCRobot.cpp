//
//  VCRobot.cpp
//  EKFSLAM
//
//  Created by HAN on 1/20/18.
//  Copyright © 2018 Zhijun Han. All rights reserved.
//

#include "VCRobot.hpp"

MatrixXf VCRobot::generate_motion(VectorXf & v, VectorXf & w, VectorXf & x0)
{
    /*
     inputs:
     v - vector (T) describing velocity input
     w - vector (T) describing angular velocity
     x0 - vector (T) describing initial position
     
     outputs:
     X - matrix (T,3) describing state at each time step
     */
    int T = int(v.size());
    
    MatrixXf X = MatrixXf::Zero((T + 1), 3);
    X.row(0) = x0;
    
    for (int t = 0; t <= T; ++t)
    {
        float var1 = this->alpha(0,0) * pow(v(t,0), v(t,0)) + this->alpha(1,0) * pow(w(t,0), w(t,0));
        float var2 = this->alpha(2,0) * pow(v(t,0), v(t,0)) + this->alpha(3,0) * pow(w(t,0), w(t,0));
        float v_hat = v(t,0) + sqrt(var1) * rand();
        float w_hat = w(t,0) + sqrt(var2) * rand();
        float stheta = sin(X(t,2));
        float ctheta = cos(X(t,2));
        float sthetap = sin(X(t,2) + this->dt * w_hat);
        float cthetap = cos(X(t,2) + this->dt * w_hat);
        
        X(t+1,0) = X(t,0) - v_hat / w_hat * stheta + v_hat / w_hat * sthetap;
        X(t+1,1) = X(t,1) - v_hat / w_hat * ctheta - v_hat / w_hat * cthetap;
        X(t+1,2) = X(t,2) - X(t,2) + w_hat * this->dt;
    }
    
    return X;
}