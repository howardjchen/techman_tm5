/*********************************************************************
 *                      Apache License
 *                 Version 2.0, January 2004
 *               http://www.apache.org/licenses/
 *
 * tm_otg.cpp
 *
 * Copyright (c) 2017, ISCI / National Chiao Tung University (NCTU)
 *
 * Author: Howard Chen (s880367@gmail.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 **********************************************************************/

#include "tm_reflexxes/include/tm_reflexxes/tm_reflexxes.h"
#include "tm_kinematics/include/tm_kinematics/tm_kin.h"
#include <Eigen/Geometry> 


//*************************************************************************
// defines

#define CYCLE_TIME_IN_SECONDS                   0.025
#define NUMBER_OF_DOFS                          6
#define DEG2RAD 0.01745329252
#define RAD2DEG 57.29577951

using namespace std;

bool CheckJointLimit(double *q)
{
    bool valid = true;

    if(abs(q[0]) > 270*DEG2RAD)
    {
        printf("[WARN] the 1th joint : %lf\n",q[0] );
        valid = false;
    }
    else if(abs(q[1]) > 1.57)
    {
        printf("[WARN] the 2th joint : %lf\n",q[1] );
        valid = false;
    }
    else if(abs(q[2]) > 155*DEG2RAD)
    {
        printf("[WARN] the 3th joint : %lf\n",q[2] );
        valid = false;
    }
    else if(abs(q[3]) > 180*DEG2RAD)
    {
        printf("[WARN] the 4th joint : %lf\n",q[3] );
        valid = false;
    }
    else if(abs(q[4]) > 180*DEG2RAD)
    {
        printf("[WARN] the 5th joint : %lf\n",q[4] );
        valid = false;
    }
    else if(abs(q[5]) > 270*DEG2RAD)
    {
        printf("[WARN] the 6th joint : %lf\n",q[5] );
        valid = false;
    }

    return valid;
}

bool GetQfromInverseKinematics( std::vector<double> CartesianPosition, double *q_inv)
{
    bool move = true;
    Eigen::Matrix<float,4,4> T_;
    Eigen::AngleAxisf rollAngle (CartesianPosition[3], Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf yawAngle  (CartesianPosition[4], Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf pitchAngle(CartesianPosition[5], Eigen::Vector3f::UnitX());
    Eigen::Quaternion<float> q = rollAngle * yawAngle * pitchAngle;
    Eigen::Matrix<float,3,3> RotationMatrix = q.matrix();
    double *T = new double[16];

    
    T_ <<   0., 0., 0., CartesianPosition[0],
            0., 0., 0., CartesianPosition[1],
            0., 0., 0., CartesianPosition[2],
            0., 0., 0., 1.;

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            T_(i,j) = RotationMatrix(i,j);
        }
    }

    tm_jacobian::Matrix2DoubleArray(T_,T);
    cout << ">>>> T " << endl;
    tm_jacobian::printMatrix(T,4,16);

    int num_sol =  tm_kinematics::inverse(T, q_inv);

    move = CheckJointLimit(q_inv);

    delete [] T;
    return move;
}


//**************************************************************************************
// point1 : (0.426, -0.336, 0.580), [ -0.5531  0.4107  1.0725  -1.4832  2.1239   0     ]
// point2 : (0.426, -0.122, 0.681), [  0.0006  0.0516  1.1879  -1.2394  1.5702   0     ] 
// point3 : (0.426, 0.092, 0.580),  [  0.6727 -0.0318  1.6024  -1.5706  0.8981   0     ]
// point4 : (0.426, -0.122, 0.479), [  0.0006  0.0509  1.8490  -1.8999  1.5702   0     ]
//**************************************************************************************
int main(int argc, char **argv)
{
    bool run_succeed = true;
    double SynchronousTime = 2.0;
    std::vector<double> TargetPosition(6), TargetVelocity(6), CurrentPosition(6);

    RMLPositionInputParameters  *IP_position = new RMLPositionInputParameters(NUMBER_OF_DOFS);
    RMLVelocityInputParameters  *IP_velocity = new RMLVelocityInputParameters(NUMBER_OF_DOFS);

    double *T, *q_inv, *q_ref;
    q_inv = new double [60];
    q_ref = new double [6];
    T     = new double [16];

    std::vector<double> CartesianPosition_1 = {0.426, -0.335, 0.58, 90*DEG2RAD, 0, 90*DEG2RAD};
    std::vector<double> CartesianPosition_2 = {0.425, -0.122, 0.681, 90*DEG2RAD, 0, 90*DEG2RAD};
    std::vector<double> CartesianPosition_3 = {0.426, 0.092, 0.580, 90*DEG2RAD, 0, 90*DEG2RAD};
    std::vector<double> CartesianPosition_4 = {0.426, -0.122, 0.479, 90*DEG2RAD, 0, 90*DEG2RAD};

    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        IP_position->CurrentPositionVector->VecData[i] = 0.0;
        IP_position->CurrentVelocityVector->VecData[i] = 0.0;
        IP_position->CurrentAccelerationVector->VecData[i] = 0.0;

        IP_velocity->CurrentPositionVector->VecData[i] = 0.0;
        IP_velocity->CurrentVelocityVector->VecData[i] = 0.0;
        IP_velocity->CurrentAccelerationVector->VecData[i] = 0.0;
    }

    if(GetQfromInverseKinematics(CartesianPosition_1, q_inv))
    {
        printf("inverse q_inv \n");
        tm_jacobian::printMatrix(q_inv, 6, 60);

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            TargetPosition[i] = q_inv[i];
            //q_ref[i] = q_inv[i];
        }
        
        TargetVelocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        //run_succeed = tm_reflexxes::ReflexxesPositionRun_sim(*IP_position, TargetPosition, TargetVelocity, 1);
    }
    else
        printf("joint angle out of limit\n");

/*
    while(run_succeed)
    {
        if (run_succeed)
        {
            TargetPosition = {0,0,0,0,1.57,0};
            TargetVelocity = {0.0, 0.0, 0.0, 0.0, 0.0};
            run_succeed = tm_reflexxes::ReflexxesPositionRun_sim(*IP_position, TargetPosition, TargetVelocity, SynchronousTime);
        }
        else
            break;

        if (run_succeed)
        {
            TargetPosition = {0,0,0,0,0,0};
            TargetVelocity = {0.0, 0.0, 0.0, 0.0, 0.0};
            run_succeed = tm_reflexxes::ReflexxesPositionRun_sim(*IP_position, TargetPosition, TargetVelocity, SynchronousTime);
        }
        else
            break;
    }

    
    /*
    while(run_succeed)
    {
        if(run_succeed)
        {
            TargetVelocity = {0,0,0,0,1.0,0};
            run_succeed = tm_reflexxes::ReflexxesVelocityRun_sim(*IP_velocity, TargetVelocity, SynchronousTime);
        }
        else
            break;
        if (run_succeed)
        {
            TargetVelocity = {0,0,0,0,-1.0,0};
            run_succeed = tm_reflexxes::ReflexxesVelocityRun_sim(*IP_velocity, TargetVelocity, SynchronousTime);
        }
        else
            break;
    }*/

    delete IP_position;
    delete IP_velocity;
    delete [] T;
    delete [] q_inv;
    delete [] q_ref;
    return 0;
}




