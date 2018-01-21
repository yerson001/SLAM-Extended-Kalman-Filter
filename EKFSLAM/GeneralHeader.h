//
//  GeneralHeader.h
//  EKFSLAM
//
//  Created by HAN on 1/20/18.
//  Copyright © 2018 Zhijun Han. All rights reserved.
//

#ifndef GeneralHeader_h
#define GeneralHeader_h

#include <iostream>
#include <stdio.h>

#include "Eigen/Dense"
#include <vector>
#include <string>
#include <math.h>
#include <random>

#include "EKFSLAM.h"

#define PI 3.1416f
#define inf 1e15
#define N_landmarks 50

using namespace std;
using Eigen::MatrixXf;
using Eigen::VectorXf;
using Eigen::RowVectorXf;
//using std::vector;

#endif /* GeneralHeader_h */
