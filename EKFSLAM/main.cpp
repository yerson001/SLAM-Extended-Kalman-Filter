//
//  main.cpp
//  EKFSLAM
//
//  Created by HAN on 1/14/18.
//  Copyright © 2018 Zhijun Han. All rights reserved.
//

#include "GeneralHeader.h"
#include "LandmarkSensor.hpp"
#include "EKFSLAM.hpp"
#include "VCRobot.hpp"

int main(int argc, const char * argv[]) {
    float dt = 0.1;
    VectorXf alpha(4,1);
    alpha << 0.1, 0.01, 0.01, 0.1;
    
    VectorXf t;
    for (int i = 0; i < 400; ++i)
    {
        t(i) = i * dt;
    }
    
    VectorXf x0(1,3);
    x0 << -5, -3, PI / 2;
    
    //int N_landmarks = 50;
    
    MatrixXf Y(N_landmarks,3);
    VectorXf Y_colm0, Y_colm1, Y_colm2;
    
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> dis(-20.0, 20.0);
    for (int n = 0; n < N_landmarks; ++n)
    {
        Y_colm0(n,0) = dis(gen);
        Y_colm1(n,0) = dis(gen);
        Y_colm2(n,0) = dis(gen);
    }
    
    //Y << Y_colm0, Y_colm1, Y_colm2;
    Y.col(0) = Y_colm0;
    Y.col(1) = Y_colm1;
    Y.col(2) = Y_colm2;
    
    MatrixXf R(3,3);
    R << 0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0001;
    //Eigen::Matrix3f R(0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0001);
    
    VectorXf v, w;
    for (int n = 0; n < t.size(); ++n)
    {
        v(n,0) = 1 + 0.5 * cos(0.4 * PI * t(n,0));
        w(n,0) = -0.2 + 2 * cos(1.2 * PI * t(n,0));
    }
    
    VCRobot robot(dt, alpha);
    MatrixXf X = robot.generate_motion(v, w, x0);
    
    LandmarkSensor rbsensor(Y, R, float(PI/4));
    
    MatrixXf U;
    U.col(0) = v;
    U.col(1) = w;
    
    VectorXf x_hat;
    x_hat << x0;
    
    VectorXf X_hat, Cov_hat;
    EKFSLAM ekf(x0, R, dt, alpha);
    
    vector<VectorXf> result_mu;
    vector<MatrixXf> result_cov;
    
    for (int t = 0; t < U.rows(); ++t)
    {
        RowVectorXf x_exe(1,3);
        x_exe << X.row(t);
        MatrixXf z = rbsensor.simulate(x_exe);
        VectorXf u = U.row(t);
        pair<VectorXf, VectorXf> result = ekf.filter(z, u);
        
        result_mu.push_back(result.first);
        result_cov.push_back(result.second);
    }
    
    return 0;
    
}
