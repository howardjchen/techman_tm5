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


//*************************************************************************
// defines

#define CYCLE_TIME_IN_SECONDS                   0.025
#define NUMBER_OF_DOFS                          6
#define DEG2RAD 0.01745329252
#define RAD2DEG 57.29577951

#define MAX_VELOCITY 1.0
#define MAX_ACC 0.0375*40 // 0.0375 : acc in 25ms

using namespace std;

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

    double *test = new double [10];

    double position1[3] = {0.426, -0.335, 0.580};
    double position2[3] = {0.425, -0.122, 0.681};
    double position3[3] = {0.426, 0.092, 0.580};
    double position4[3] = {0.426, -0.122, 0.479};

    Eigen::Matrix4f T_;
    T_ <<  -0.,     0.,     1.,     0., 
            1.,     0.,     0.,     0., 
            0.,     1.,     0.,     0., 
            0.,     0.,     0.,     1.; 

    //***********************
    // point1 : (0.426, -0.336, 0.580), [ -0.5531  0.4107  1.0725  -1.4832  2.1239  -0.0000]
    // point2 : (0.426, -0.122, 0.681), [  0.0006  0.0516  1.1879  -1.2394  1.5702   0     ] [0.0006 1.2025 -1.1879 -0.0147 1.5702 0]
    // point3 : (0.426, 0.092, 0.580),  [0.6727 -0.0318 1.6024 -1.5706 0.8981 0]
    // point4 : (0.426, -0.122, 0.479), [0.0006 0.0509 1.8490 -1.8999 1.5702 0]
    //***********************

    q_ref[0] = 0;
    q_ref[1] = 0;
    q_ref[2] = 1.5708;
    q_ref[3] = -1.5708;
    q_ref[4] = 1.5708;
    q_ref[5] = 0;

    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        IP_position->CurrentPositionVector->VecData[i] = 0.0;
        IP_position->CurrentVelocityVector->VecData[i] = 0.0;
        IP_position->CurrentAccelerationVector->VecData[i] = 0.0;

        IP_velocity->CurrentPositionVector->VecData[i] = 0.0;
        IP_velocity->CurrentVelocityVector->VecData[i] = 0.0;
        IP_velocity->CurrentAccelerationVector->VecData[i] = 0.0;
    }

    for (int j = 0; j < 4; ++j)
    {
        if(j == 0)
        {
            T_(0,3) = position1[0];
            T_(1,3) = position1[1];
            T_(2,3) = position1[2];
        }
        else if(j == 1)
        {
            T_(0,3) = position2[0];
            T_(1,3) = position2[1];
            T_(2,3) = position2[2];
        }
        else if(j == 2)
        {
            T_(0,3) = position3[0];
            T_(1,3) = position3[1];
            T_(2,3) = position3[2];
        }
        else if(j == 3)
        {            
            T_(0,3) = position4[0];
            T_(1,3) = position4[1];
            T_(2,3) = position4[2];
        }
        else
        {
            T_(0,3) = position4[0];
            T_(1,3) = position4[1];
            T_(2,3) = position4[2];
        }


        tm_jacobian::Matrix2DoubleArray(T_,T);
        cout << ">>>> T " << endl;
        tm_jacobian::printMatrix(T,4,16);

        int num_sol =  tm_kinematics::inverse(T, q_inv, q_ref);

        printf("inverse q_inv with %d solutions in rad, %d\n",num_sol,j );
        tm_jacobian::printMatrix(q_inv, 6, 60);


        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            TargetPosition[i] = q_inv[i];
            q_ref[i] = q_inv[i];
        }
        
        TargetVelocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        run_succeed = tm_reflexxes::ReflexxesPositionRun_sim(*IP_position, TargetPosition, TargetVelocity, SynchronousTime);
    
    }

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
*/
    
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




