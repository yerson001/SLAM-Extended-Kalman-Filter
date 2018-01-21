//
//  EKFSLAM.cpp
//  EKFSLAM
//
//  Created by HAN on 1/20/18.
//  Copyright © 2018 Zhijun Han. All rights reserved.
//

#include "EKFSLAM.hpp"

VectorXf EKFSLAM::f(VectorXf & x, VectorXf & u)
{
    VectorXf xp = VectorXf::Zero(x.rows(), x.cols());
    float v = u(0);
    float w = u(1);
    float stheta = sin(x(2));
    float ctheta = cos(x(2));
    float sthetap = sin(x(2) + this->dt * w);
    float cthetap = cos(x(2) + this->dt * w);
    xp(0) = x(0) - v / w * stheta + v / w * sthetap;
    xp(1) = x(1) - v / w * ctheta + v / w * cthetap;
    xp(2) = x(2) + w * this->dt;
    
    return xp;
}

VectorXf EKFSLAM::h(VectorXf & x, VectorXf & y, VectorXf & u)
{
    VectorXf zp = VectorXf::Zero(3);
    
    zp(0) = sqrt(pow((x(0) - y(0)), 2) + pow((x(1) - y(1)),2));
    zp(1) = atan2(y(1) - x(1), y(0) - x(0)) - x(2);
    
    if (zp(1) > PI)
        zp(1) -= 2 * PI;
    else if(zp(1) < -PI)
        zp(1) += 2 * PI;
    
    zp(2) = y(2);
    return zp;
}

VectorXf EKFSLAM::F(VectorXf & x, VectorXf & u)
{
    long int n = x.rows();
    float v = u(0);
    float w = u(1);
    float stheta = sin(x(2));
    float ctheta = cos(x(2));
    float sthetap = sin(x(2) + this->dt * w);
    float cthetap = cos(x(2) + this->dt * w);
    
    MatrixXf F = MatrixXf::Zero(n, n);
    F(0,2) = -v / w * ctheta + v / w * cthetap;
    F(1,2) = -v / w * stheta + v / w * sthetap;
    return F;
    
}

MatrixXf EKFSLAM::G(VectorXf & x, VectorXf & u)
{
    long int n = x.rows();
    long int k = u.rows();
    float v = u(0);
    float w = u(1);
    float stheta = sin(x(2));
    float ctheta = cos(x(2));
    float sthetap = sin(x(2) + this->dt * w);
    float cthetap = cos(x(2) + this->dt * w);
    
    MatrixXf G = MatrixXf::Zero(n, k);
    G(0,0) = (-stheta + sthetap) / w;
    G(0,1) = v * (stheta - sthetap) / (w * w) + v *(ctheta * this->dt) / w;
    G(1,0) = (ctheta - cthetap ) / w;
    G(1,1) = -v * (ctheta - cthetap) / (w * w) + v * (stheta * this->dt) / w;
    G(2,1) = this->dt;
    return G;
}

MatrixXf EKFSLAM::H(VectorXf & x, VectorXf & y, VectorXf & u)
{
    MatrixXf H = MatrixXf::Zero(3, x.cols());
    float dx = y(0) - x(0);
    float dy = y(1) - x(1);
    float q = pow((y(0) - x(0)), 2) + pow((y(1) - x(1)), 2);
    float sq = sqrt(q);
    H(0,0) = -(dx) / sq;
    H(0,1) = -(dy) / sq;
    H(0,2) = 0;
    H(0,3) = dx / sq;
    H(0,4) = dy / sq;
    H(0,5) = 0;
    H(1,0) = dy / q;
    H(1,1) = -dx / q;
    H(1,2) = -1;
    H(1,3) = -dy / q;
    H(1,4) = dx / q;
    H(1,5) = 0;
    H(2,5) = 1;
    return H;
}

MatrixXf EKFSLAM::Q(VectorXf & u)
{
    long int k = u.rows();
    float v = u(0);
    float w = u(1);
    MatrixXf Q = MatrixXf::Zero(k, k);
    Q(0,0) = alpha(0) * v * v + alpha(1) * w * w;
    Q(1,1) = alpha(2) * v * v + alpha(3) * w * w;
    return Q;
}

pair<VectorXf, MatrixXf> EKFSLAM::filter(MatrixXf & z, VectorXf & u)
{
    /*
     z - (N_landmarks_seen x 3)
     u - control inputs
     Filters the  slam problem over 1 time step
     */
    
    //Prediction
    
    VectorXf mu_arg(3,1);
    mu_arg << this->mu.head(3);
    
    this->mu.head(3) = this->f(mu, u);
    MatrixXf Fx = MatrixXf::Zero(3, 3*N_landmarks+3);
    Fx.block<3,3>(0,0) = MatrixXf::Identity();
    
    MatrixXf F, I;
    VectorXf G;
    
    I = MatrixXf::Identity(3*N_landmarks+3, 3*N_landmarks+3);
    
    mu_arg << this->mu.head(3);
    
    F = I + Fx.transpose() * this->F(mu_arg, u) * Fx;
    G = 5 * this->G(mu_arg, u);
    
    this->cov = F * this->cov * F.transpose() + Fx.transpose() * G * this->Q(u) * G.transpose() * Fx;
    
    if (z.size() == 0)
    {
        return {this->mu, this->cov};
    }
    
    // Measurement Update
    
    VectorXf l;
    l << z.col(2);
    for (int j = 0; j < l.size(); ++j)
    {
        long int s = l(j);
        
        if (this->seen(s) == false)
        {
            this->mu(3*(s+1)) = this->mu(0) + z(0) * cos(z(1) + this->mu(2));
            this->mu(3*(s+1) + 1) = this->mu(1) + z(0) * sin(z(1) + this->mu(2));
            this->mu(3*(s+1) + 2) = s;
            this->seen(s) = true;
        }
        VectorXf y;
        y << this->mu.segment(3*(s+1), 3*(s+1)+3);
        
        VectorXf z_hat;
        
        mu_arg << this->mu.head(3);
        z_hat << this->h(mu_arg, y, u);
        MatrixXf Fxj = MatrixXf::Zero(6, 3*N_landmarks+3);
        
        Fxj.block<3,3>(0,0) = MatrixXf::Identity();
        Fxj(3,3*(s+1)) = 1;
        Fxj(4,3*(s+1)+1) = 1;
        Fxj(5,3*(s+1)+2) = 1;
        
        MatrixXf H = this->H(mu_arg, y, u) * Fxj;
        MatrixXf K = this->cov * H.transpose() * ((H * this->cov * H.transpose() + this->R).inverse());
        
        VectorXf innovation = z - z_hat;
        if (innovation(1) > PI)
            innovation(1) -= 2 * PI;
        else if (innovation(1) < -PI)
            innovation(1) += 2 * PI;
        
        innovation = innovation.transpose();
        MatrixXf update_mat = K * innovation;
        
        // To flatten the measurement update to array data structure
        Eigen::Map<RowVectorXf> update(update_mat.data(), update_mat.size());
        
        this->mu += update;
        MatrixXf eye = MatrixXf::Identity(3*N_landmarks+3, 3*N_landmarks+3);
        
        this->cov << (eye - K * H).dot(this->cov);
    }
    
    pair<VectorXf, MatrixXf> result;
    
    result = {this->mu, this->cov};
    return result;
}
