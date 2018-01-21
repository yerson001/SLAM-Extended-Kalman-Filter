//
//  EKFSLAM.hpp
//  EKFSLAM
//
//  Created by HAN on 1/20/18.
//  Copyright © 2018 Zhijun Han. All rights reserved.
//

#ifndef EKFSLAM_hpp
#define EKFSLAM_hpp

#include "GeneralHeader.h"

class EKFSLAM {
    /*
     Class to implement an Extended Kalman Filter SLAM
     for a system with noise on control inputs
     */
private:
    /*
     Class variables:
     f(x,u) - function handle nonlinear state transition
     h(x,u) - function handle nonlinear measurement model
     F(x,u) - function handle Jacobian of state w/ respect to state
     G(x,u) - function handle Jacobian of state w/ respect to input
     H(x,u) - function handle Jacobian of measurement w/ respect to state
     Q(u) - function handle Covariance of input
     R - matrix (mxm) sensor noise covariance
     */
    VectorXf mu, seen, alpha;
    MatrixXf cov, R;
    float dt;
    
public:
    EKFSLAM();
    
    EKFSLAM(VectorXf & x0, MatrixXf & R, float & dt, VectorXf & alpha)
    {
        this->mu = VectorXf::Zero(3*N_landmarks+3,1);
        this->mu.row(0) = x0;
        this->cov = inf * (MatrixXf::Identity(3*N_landmarks+3, 3*N_landmarks+3));
        this->cov.topLeftCorner(3,3) << 0,0,0,0,0,0,0,0,0;
        this->R = R;
        this->seen = VectorXf::Constant(N_landmarks, false);
        
        this->dt = dt;
        this->alpha = alpha;
    }
    
    VectorXf f(VectorXf &, VectorXf &);
    
    VectorXf h(VectorXf &, VectorXf &, VectorXf &);
    
    VectorXf F(VectorXf &, VectorXf &);
    
    MatrixXf G(VectorXf &, VectorXf &);
    
    MatrixXf H(VectorXf &, VectorXf &, VectorXf &);
    
    MatrixXf Q(VectorXf &);
    
    pair<VectorXf, MatrixXf> filter(MatrixXf &, VectorXf &);
    
};


#endif /* EKFSLAM_hpp */
