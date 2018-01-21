//
//  VCRobot.hpp
//  EKFSLAM
//
//  Created by HAN on 1/20/18.
//  Copyright © 2018 Zhijun Han. All rights reserved.
//

#ifndef VCRobot_hpp
#define VCRobot_hpp

#include "GeneralHeader.h"

class VCRobot {
    /*
     Class to implement a velocity controlled two-wheeled robot
     
     The robot has state (x, y, theta)
     The motion model is described in Probabilistic Robotics by Thrun Ch. 5d
     */
    
public:
    /*
     Constructor.
     */
    VCRobot();
    
    VCRobot(float& dt, VectorXf & alpha)
    {
        /*
         member variables:
         dt - time step
         alpha - vector (4) describing noise characteristics
         */
        this->dt = dt;
        this->alpha = alpha;
    }
    
    /*
     Destructor.
     */
    //~VCRobot();
    
    MatrixXf generate_motion(VectorXf &, VectorXf &, VectorXf &);
    
private:
    // time step
    float dt;
    
    // Noise characteristics
    VectorXf alpha;
    
};

#endif /* VCRobot_hpp */
