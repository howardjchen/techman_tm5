/*********************************************************************
 *                      Apache License
 *                 Version 2.0, January 2004
 *               http://www.apache.org/licenses/
 *
 * tm_pbd.cpp
 *
 * Copyright (c) 2017, ISCI / National Chiao Tung University (NCTU)
 *
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
/**
 * Authors:
 * 1. Howard Chen (s880367@gmail.com)
 * 2. Evansong    (https://github.com/evansong0307)
**/

#include <mysql/mysql.h>
#include "tm_reflexxes/include/tm_reflexxes/tm_reflexxes.h"
#include "tm_kinematics/include/tm_kinematics/tm_kin.h"
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h>

//*************************************************************************
// defines

#define CYCLE_TIME_IN_SECONDS                   0.025
#define NUMBER_OF_DOFS                          6
#define DEG2RAD 0.01745329252
#define RAD2DEG 57.29577951
#define PITCH 0
#define CompenX 0.1
#define CompenY 0.15
#define CompenZ 0.458

using namespace std;


void finish_with_error(MYSQL *con)
{
    fprintf(stderr, "%s\n", mysql_error(con));
    mysql_close(con);
    exit(1);
}




/* tm_moder_driver.cpp */
/* Initialize new terminal i/o settings */
void initTermios(int echo)
{
    tcgetattr(STDIN_FILENO, &oldt); /* grab old terminal i/o settings */
    newt = oldt; /* make new settings same as old settings */
    newt.c_lflag &= ~ICANON; /* disable buffered i/o */
    newt.c_lflag &= echo ? ECHO : ~ECHO; /* set echo mode */
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); /* use these new terminal i/o settings now */
}
/* Restore old terminal i/o settings */
void resetTermios()
{
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}
int kbhit()
{
    struct timeval tv;
    fd_set rdfs;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&rdfs);
    FD_SET (STDIN_FILENO, &rdfs);
    select(STDIN_FILENO + 1, &rdfs, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &rdfs);
}

void print_vectord(const std::vector<double>& vec)
{
    for (int i = 0; i < vec.size() - 1; i++)
    {
        printf("%.4f, ", vec[i]);
    }
    printf("%.4f", vec[vec.size() - 1]);
}


void print_rt_1(const TmDriver& TR, double& time_s, std::vector<double>& vec)
{
    time_s = TR.interface->stateRT->getQAct(vec);
    printf("[ INFO]  q_act:=< ");
    print_vectord(vec);
    printf(" > [%.3f]\n", time_s);
    time_s = TR.interface->stateRT->getQCmd(vec);
    printf("[ INFO]  q_cmd:=< ");
    print_vectord(vec);
    printf(" > [%.3f]\n", time_s);
}
void print_rt_2(const TmDriver& TR, double& time_s, std::vector<double>& vec)
{
    time_s = TR.interface->stateRT->getQdAct(vec);
    printf("[ INFO] qd_act:=< ");
    print_vectord(vec);
    printf(" > [%.3f]\n", time_s);
    time_s = TR.interface->stateRT->getQdCmd(vec);
    printf("[ INFO] qd_cmd:=< ");
    print_vectord(vec);
    printf(" > [%.3f]\n", time_s);
}
void print_rt_3(const TmDriver& TR, double& time_s, std::vector<double>& vec)
{
    time_s = TR.interface->stateRT->getQtAct(vec);
    printf("[ INFO] qt_act:=< ");
    print_vectord(vec);
    printf(" > [%.3f]\n", time_s);
    time_s = TR.interface->stateRT->getQtCmd(vec);
    printf("[ INFO] qt_cmd:=< ");
    print_vectord(vec);
    printf(" > [%.3f]\n", time_s);
}
void print_rt_4(const TmDriver& TR, double& time_s, std::vector<double>& vec)
{
    time_s = TR.interface->stateRT->getTool0PosAct(vec);
    printf("[ INFO] tool0_pos_act:=< ");
    print_vectord(vec);
    printf(" > [%.3f]\n", time_s);
    time_s = TR.interface->stateRT->getTool0VelAct(vec);
    printf("[ INFO] tool0_vel_act:=< ");
    print_vectord(vec);
    printf(" > [%.3f]\n", time_s);
}
void print_rt_5(const TmDriver& TR, double& time_s, std::vector<double>& vec)
{
    time_s = TR.interface->stateRT->getToolPosAct(vec);
    printf("[ INFO]  tool_pos_act:=< ");
    print_vectord(vec);
    printf(" > [%.3f]\n", time_s);
    time_s = TR.interface->stateRT->getToolPosCmd(vec);
    printf("[ INFO]  tool_pos_cmd:=< ");
    print_vectord(vec);
    printf(" > [%.3f]\n", time_s);
}
void print_rt_6(const TmDriver& TR, double& time_s, std::vector<double>& vec)
{
    time_s = TR.interface->stateRT->getToolVelAct(vec);
    printf("[ INFO]  tool_vel_act:=< ");
    print_vectord(vec);
    printf(" > [%.3f]\n", time_s);
    time_s = TR.interface->stateRT->getToolVelCmd(vec);
    printf("[ INFO]  tool_vel_cmd:=< ");
    print_vectord(vec);
    printf(" > [%.3f]\n", time_s);
}
void print_rt_7(const TmDriver& TR, double& time_s, std::vector<double>& vec)
{
    double val;
    bool isRA, isAE, isUW;
    time_s = TR.interface->stateRT->getTcpForce(vec);
    printf("[ INFO] tcp_force_est:=< ");
    print_vectord(vec);
    printf(" > [%.3f], ", time_s);
    time_s = TR.interface->stateRT->getTcpForceNorm(val);
    printf("norm:=%.4f  [%.3f]\n", val, time_s);
    TR.interface->stateRT->getKineConfig(isRA, isAE, isUW);
    //snprintf(_msg, 256, "kine_config:=< %d, %d, %d >", (int)isRA, (int)isAE, (int)isUW);
    print_info("kine_config:=< %d, %d, %d >", (int)isRA, (int)isAE, (int)isUW);
    val = TR.interface->stateRT->getSpdDownRatio();
    //snprintf(_msg, 256, "spd_down_ratio:=%.4f", val);
    print_info("spd_down_ratio:=%.4f", val);
    val = TR.interface->stateRT->getSpdJRatio();
    printf("[ INFO] spd_j: ratio:=%.4f, ", val);
    val = TR.interface->stateRT->getSpdJTa();
    printf("Ta:=%.4f\n", val);
    val = TR.interface->stateRT->getSpdLRatio();
    printf("[ INFO] spd_l: spd:=%.4f, ", val);
    val = TR.interface->stateRT->getSpdLTa();
    printf("Ta:=%.4f\n", val);
}
void print_rt_8(const TmDriver& TR)
{
    std::vector<bool> vec;
    TR.interface->stateRT->getDigitalInputMB(vec);
    printf("[ INFO] MB DI: ");
    for (int i = 0; i < vec.size() - 1; i++)
    {
        printf("%d, ", (int)vec[i]);
    }
    printf("%d\n", (int)vec[vec.size() - 1]);
    TR.interface->stateRT->getDigitalOutputMB(vec);
    printf("[ INFO] MB DO: ");
    for (int i = 0; i < vec.size() - 1; i++)
    {
        printf("%d, ", (int)vec[i]);
    }
    printf("%d\n", (int)vec[vec.size() - 1]);
    TR.interface->stateRT->getDigitalInputEE(vec);
    printf("[ INFO] EE DI: ");
    for (int i = 0; i < vec.size() - 1; i++)
    {
        printf("%d, ", (int)vec[i]);
    }
    printf("%d\n", (int)vec[vec.size() - 1]);
    TR.interface->stateRT->getDigitalOutputEE(vec);
    printf("[ INFO] EE DO: ");
    for (int i = 0; i < vec.size() - 1; i++)
    {
        printf("%d, ", (int)vec[i]);
    }
    printf("%d\n", (int)vec[vec.size() - 1]);
}
void print_rt_9(const TmDriver& TR, double& time_s)
{
    bool isErr;
    unsigned char ErrCode;
    print_info("robot_mode:=%d, safety_mode:=%d, teach_mode:=%d, control_mode:=%d",
               TR.interface->stateRT->getRobotMode(), TR.interface->stateRT->getSafetyMode(),
               TR.interface->stateRT->getTeachMode(), TR.interface->stateRT->getControlMode()
              );
    print_info("QueueCmdCount:=%d, BuffEmptyFlag:=%d",
               TR.interface->stateRT->getQueCmdCount(),
               TR.interface->stateRT->getBufEmptyFlag()
              );
    isErr = TR.interface->stateRT->getError(ErrCode, time_s);
    print_info("ErrorCode:=[%d][0x%x] [%.3f]", (int)isErr, ErrCode, time_s);
}

std::vector<double> parse_cmd(char* cstr, const char* delim, double& res)
{
    std::vector<double> ret;
    //int count = 0;
    char* pch;
    char* pch_save;
    pch = strtok_r(cstr, delim, &pch_save);
    //printf("%d: %s\n", count, pch);
    if (pch != NULL)
    {
        while ((pch = strtok_r(NULL, delim, &pch_save)) != NULL)
        {
            //count++;
            if (ret.size() < 6)
            {
                ret.push_back(atof(pch));
            }
            else
            {
                res = atof(pch);
                break;
            }
            //printf("%d: %s\n", count, pch);
        }
    }

    return ret;
}
/* tm_moder_driver.cpp */

void ComplianceTeach(TmDriver& TM5)
{
    bool run_succeed = true;
    double SynchronousTime = 5.0;
    std::vector<double> TargetPosition, TargetVelocity, CurrentPosition;
    std::vector<double> T_positioin_1, T_positioin_2, T_positioin_3;
    T_positioin_1 = {0, 0, 0, 0, 0, 0};
    T_positioin_2 = {0, 0, 0, 0, 0, 0};
    T_positioin_3 = {0, 0, 0, 0, 0, 0};

    RMLPositionInputParameters  *IP_position = new RMLPositionInputParameters(NUMBER_OF_DOFS);
    RMLVelocityInputParameters  *IP_velocity = new RMLVelocityInputParameters(NUMBER_OF_DOFS);


    getchar();
    TM5.interface->stateRT->getQAct(T_positioin_1);
    for (int i = 0; i < 6; ++i)
    {
        printf("%10.4lf \n", T_positioin_1[i]);
    }


    getchar();
    TM5.interface->stateRT->getQAct(T_positioin_2);
    for (int i = 0; i < 6; ++i)
    {
        printf("%10.4lf \n", T_positioin_2[i]);
    }

    getchar();
    TM5.interface->stateRT->getQAct(T_positioin_3);
    for (int i = 0; i < 6; ++i)
    {
        printf("%10.4lf \n", T_positioin_3[i]);
    }



    getchar();
    cout << "motion start" << endl;
    TM5.interface->stateRT->getQAct(CurrentPosition);
    TM5.setJointSpdModeON();
    cout << "joint speed mode turn on" << endl;

    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        IP_position->CurrentPositionVector->VecData[i] = CurrentPosition[i];
        IP_position->CurrentVelocityVector->VecData[i] = 0.0;
        IP_position->CurrentAccelerationVector->VecData[i] = 0.0;

        IP_velocity->CurrentPositionVector->VecData[i] = CurrentPosition[i];
        IP_velocity->CurrentVelocityVector->VecData[i] = 0.0;
        IP_velocity->CurrentAccelerationVector->VecData[i] = 0.0;
    }

    while (run_succeed)
    {
        if (run_succeed)
        {
            TargetVelocity = {0.0, 0.0, 0.0, 0.0, 0.0};
            run_succeed = tm_reflexxes::ReflexxesPositionRun(TM5, *IP_position, T_positioin_1, TargetVelocity, SynchronousTime);
        }
        else
            break;

        if (run_succeed)
        {
            TargetVelocity = {0.0, 0.0, 0.0, 0.0, 0.0};
            run_succeed = tm_reflexxes::ReflexxesPositionRun(TM5, *IP_position, T_positioin_2, TargetVelocity, SynchronousTime);
        }
        else
            break;

        if (run_succeed)
        {
            TargetVelocity = {0.0, 0.0, 0.0, 0.0, 0.0};
            run_succeed = tm_reflexxes::ReflexxesPositionRun(TM5, *IP_position, T_positioin_3, TargetVelocity, SynchronousTime);
        }
        else
            break;
    }

    delete IP_position;
    delete IP_velocity;
}

bool CheckJointLimit(double *q)
{
    bool valid = true;

    if (abs(q[0]) > 270 * DEG2RAD)
    {
        printf("[WARN] the 1th joint : %lf\n", q[0] );
        valid = false;
    }
    else if (abs(q[1]) > 1.57)
    {
        printf("[WARN] the 2th joint : %lf\n", q[1] );
        valid = false;
    }
    else if (abs(q[2]) > 155 * DEG2RAD)
    {
        printf("[WARN] the 3th joint : %lf\n", q[2] );
        valid = false;
    }
    else if (abs(q[3]) > 180 * DEG2RAD)
    {
        printf("[WARN] the 4th joint : %lf\n", q[3] );
        valid = false;
    }
    else if (abs(q[4]) > 180 * DEG2RAD)
    {
        printf("[WARN] the 5th joint : %lf\n", q[4] );
        valid = false;
    }
    else if (abs(q[5]) > 270 * DEG2RAD)
    {
        printf("[WARN] the 6th joint : %lf\n", q[5] );
        valid = false;
    }
    else
        valid = true;

    return valid;
}

bool CheckVelocityLimit(std::vector<double> qd)
{
    bool valid = true;

    if (abs(qd[0]) > 180 * DEG2RAD || abs(qd[1]) > 180 * DEG2RAD || abs(qd[2]) > 180 * DEG2RAD)
    {
        printf("[WARN] the 1th~3th joint : %10.4lf %10.4lf %10.4lf\n", qd[0], qd[3], qd[2] );
        valid = false;
    }
    else if (abs(qd[3]) > 225 * DEG2RAD || abs(qd[4]) > 225 * DEG2RAD || abs(qd[5]) > 225 * DEG2RAD)
    {
        printf("[WARN] the 4th~6th joint : %10.4lf %10.4lf %10.4lf\n", qd[3], qd[4], qd[5] );
        valid = false;
    }
    else
        valid = true;

    return valid;
}

bool GetQfromInverseKinematics( std::vector<double> CartesianPosition, double *q_inv)
{
    Eigen::Matrix<float, 4, 4> T_;
    Eigen::AngleAxisf rollAngle (CartesianPosition[3], Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf yawAngle  (CartesianPosition[4], Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf pitchAngle(CartesianPosition[5], Eigen::Vector3f::UnitX());
    Eigen::Quaternion<float> q = rollAngle * yawAngle * pitchAngle;
    Eigen::Matrix<float, 3, 3> RotationMatrix = q.matrix();
    double *T = new double[16];


    T_ <<   0., 0., 0., CartesianPosition[0],
    0., 0., 0., CartesianPosition[1],
    0., 0., 0., CartesianPosition[2],
    0., 0., 0., 1.;

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            T_(i, j) = RotationMatrix(i, j);
        }
    }

    tm_jacobian::Matrix2DoubleArray(T_, T);
    //cout << ">>>> T " << endl;
    //tm_jacobian::printMatrix(T, 4, 16);

    int num_sol =  tm_kinematics::inverse(T, q_inv);

    delete [] T;
    return CheckJointLimit(q_inv);
}

bool GetQdfromInverseJacobian(std::vector<double> CurrentPosition, std::vector<double> EFF_Velocity, std::vector<double>& qd)
{

    Eigen::Matrix<float, 6, 1> home, q;
    home << 0, -PI * 0.5, 0, PI * 0.5, 0, 0;
    Eigen::Matrix<float, 6, 1> effspd, jointspd;

    home   << 0, -PI * 0.5, 0, PI * 0.5, 0, 0;
    effspd << EFF_Velocity[0], EFF_Velocity[1], EFF_Velocity[2], EFF_Velocity[3], EFF_Velocity[4], EFF_Velocity[5];
    q      << CurrentPosition[0], CurrentPosition[1], CurrentPosition[2], CurrentPosition[3], CurrentPosition[4], CurrentPosition[5];
    q += home;

    Eigen::Matrix<float, 6, 6> Inverse_Jacobian = tm_jacobian::Inverse_Jacobian(q);
    jointspd = Inverse_Jacobian * effspd;
    //cout << ">>>> Inverse jacobian" << endl;
    //tm_jacobian::printMatrix(Inverse_Jacobian);

    tm_jacobian::Matrix2DoubleVector(jointspd, qd);

    return CheckVelocityLimit(qd);
}

bool ReflexxesPositionRun_sim(  RMLPositionInputParameters &InputState,
                                std::vector<double> TargetPosition,
                                std::vector<double> TargetVelocity,
                                double SynTime)
{
    double time_s;
    std::vector<double> FinalPosition;
    bool pass = true;
    bool AlreadyAccese = false;

    initTermios(1);

    ReflexxesAPI *RML = NULL    ;
    RMLPositionInputParameters  *IP           = NULL;
    RMLPositionOutputParameters *OP           = NULL;
    RMLVelocityInputParameters  *IP_interrupt = NULL;
    RML          = new ReflexxesAPI(NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS);
    IP           = new RMLPositionInputParameters(NUMBER_OF_DOFS);
    OP           = new RMLPositionOutputParameters(NUMBER_OF_DOFS);
    IP_interrupt = new RMLVelocityInputParameters(NUMBER_OF_DOFS);
    RMLPositionFlags Flags;
    int ResultValue = 0;
    *IP = InputState;

    std::vector<double> PotentialField_qd(6), qd_now(6);
    double *GoalPoint = new double [3];
    double *ObstaclePoint = new double [3];
    GoalPoint[0] = 0.426;
    GoalPoint[1] = 0.092;
    GoalPoint[2] = 0.580;
    ObstaclePoint[0] = 0.426;
    ObstaclePoint[1] = -0.122;
    ObstaclePoint[2] = 0.473;

    double *q = new double [6];
    double *T = new double [16];
    double distance;

    //  ********************************************************************/
    //  Assigning all RMLPositionInputParameters :
    //  Current POS, VEL, ACC : set before call ReflexxesPositionRun
    //  Target POS, VEL       : set before call ReflexxesPositionRun
    //  Max VEL, ACC          : set after call ReflexxesPositionRun
    //  SelectionVector       : set after call ReflexxesPositionRun
    //  ********************************************************************
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        IP->MaxVelocityVector->VecData[i] = MAX_VELOCITY; //0.3247
        IP->MaxAccelerationVector->VecData[i] = MAX_ACC;
        IP->TargetPositionVector->VecData[i] = TargetPosition[i];
        IP->TargetVelocityVector->VecData[i] = TargetVelocity[i];
        IP->SelectionVector->VecData[i] = true;
    }
    //IP->MinimumSynchronizationTime = SynTime;


    if (IP->CheckForValidity())
        printf("Input values are valid!\n");
    else
        printf("Input values are INVALID!\n");


    struct timeval tm1, tm2, tm3, tm4;
    double cycle_iteration = 1.0;

    gettimeofday(&tm3, NULL);

    while (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED)
    {
        //********************************************************
        // The area execution in 25ms real time sharp

        gettimeofday(&tm1, NULL);

        ResultValue =  RML->RMLPosition(*IP, OP, Flags );

        if (ResultValue < 0)
        {
            printf("An error occurred (%d).\n", ResultValue );
            break;
        }

        //***************************************************************
        // Print out commands

        time_s = cycle_iteration * 0.025;
        cycle_iteration++;

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            q[i] = OP->NewPositionVector->VecData[i];
        }

        tm_kinematics::forward(q, T);

        printf("[ %lf ] XYZ_pos:  ", time_s);

        printf("%10.4lf %10.4lf %10.4lf ", T[3], T[7], T[11]);

        printf(" | spd: ");

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            printf("%10.4lf ", OP->NewVelocityVector->VecData[i]);
            qd_now[i] = OP->NewVelocityVector->VecData[i];
        }
        printf("\n");

        if (kbhit())
        {
            char c = getchar();
            if (c == 'q' || c == 'Q')
            {
                print_info("Smooth Stop Activate...");
                RMLVelocityInputParameters *IP_vel = new RMLVelocityInputParameters(NUMBER_OF_DOFS);

                *IP_vel->CurrentPositionVector     = *IP->CurrentPositionVector;
                *IP_vel->CurrentVelocityVector     = *IP->CurrentVelocityVector;
                *IP_vel->CurrentAccelerationVector = *IP->CurrentAccelerationVector;
                tm_reflexxes::ReflexxesSmoothStop_sim(*IP_vel, 0.5);
                delete IP_vel;
                pass = false;
                break;
            }
        }

        *IP->CurrentPositionVector =  *OP->NewPositionVector;
        *IP->CurrentVelocityVector =  *OP->NewVelocityVector;

        gettimeofday(&tm2, NULL);
        long long time_compensation = 1000000 * (tm2.tv_sec - tm1.tv_sec) + (tm2.tv_usec - tm1.tv_usec);
        usleep(24940 - time_compensation);

        //********************************************************
        // The area execution in 25ms real time sharp
    }

    gettimeofday(&tm4, NULL);
    long long tt = 1000000 * (tm4.tv_sec - tm3.tv_sec) + (tm4.tv_usec - tm3.tv_usec);

    if (pass)
    {
        printf("=============== Final state ReflexxesPositionSafetyRun_sim =========================\n");
        printf("[ %lf ]  ", time_s);
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            printf("%10.4lf ", IP->CurrentVelocityVector->VecData[i]);

        printf("\n");
        print_info("Finished in %llu us", tt);
    }
    resetTermios();
    InputState = *IP;

    delete  RML;
    delete  IP;
    delete  OP;
    delete  IP_interrupt;

    delete [] T;
    delete [] q;
    delete [] ObstaclePoint;
    delete [] GoalPoint;

    return pass;
}

void ReflexxesStart(TmDriver& TM5)
{
    TM5.setJointSpdModeON();
    print_info("joint velocity control mode ON...");

    bool run_succeed = true;
    double SynchronousTime = 5.0;
    std::vector<double> TargetPosition(6), TargetVelocity(6), CurrentPosition(6), JointVelocity(6);

    RMLPositionInputParameters  *IP_position = new RMLPositionInputParameters(NUMBER_OF_DOFS);
    RMLVelocityInputParameters  *IP_velocity = new RMLVelocityInputParameters(NUMBER_OF_DOFS);

    double *T, *q_inv, *q_ref;
    q_inv = new double [60];
    q_ref = new double [6];
    T     = new double [16];

    std::vector<double> CartesianPosition_1 = {0.426, -0.335, 0.58, 90 * DEG2RAD, 0, 90 * DEG2RAD};
    std::vector<double> CartesianPosition_2 = {0.425, -0.122, 0.681, 90 * DEG2RAD, 0, 90 * DEG2RAD};
    std::vector<double> CartesianPosition_3 = {0.426, 0.092, 0.580, 90 * DEG2RAD, 0, 90 * DEG2RAD};
    std::vector<double> CartesianPosition_4 = {0.426, -0.122, 0.479, 90 * DEG2RAD, 0, 90 * DEG2RAD};

    //CurrentPosition = {-0.5531,  0.4107,  1.0725,  -1.4832,  2.1239,   0 };
    //std::vector<double>effspd = {0,0.2378,0,0,0,0};

    TM5.interface->stateRT->getQAct(CurrentPosition);

    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        IP_position->CurrentPositionVector->VecData[i] = CurrentPosition[i];
        IP_position->CurrentVelocityVector->VecData[i] = 0.0;
        IP_position->CurrentAccelerationVector->VecData[i] = 0.0;

        IP_velocity->CurrentPositionVector->VecData[i] = CurrentPosition[i];
        IP_velocity->CurrentVelocityVector->VecData[i] = 0.0;
        IP_velocity->CurrentAccelerationVector->VecData[i] = 0.0;
    }

    TargetPosition = { -0.5531,  0.4107,  1.0725,  -1.4832,  2.1239,  -0.0000};
    TargetVelocity = {0.0, 0.0, 0.0, 0.0, 0.0};
    run_succeed = tm_reflexxes::ReflexxesPositionRun(TM5, *IP_position, TargetPosition, TargetVelocity, SynchronousTime);

    delete IP_position;
    delete IP_velocity;
    delete [] T;
    delete [] q_inv;
    delete [] q_ref;

    /*while(run_succeed)
    {
        if (run_succeed)
        {
            printf("3 -> 1 \n");
            TargetPosition = {-0.5531,  0.4107,  1.0725,  -1.4832,  2.1239,  -0.0000};
            TargetVelocity = {0.0, 0.0, 0.0, 0.0, 0.0};
            run_succeed = tm_reflexxes::ReflexxesPositionRun(TM5, *IP_position, TargetPosition, TargetVelocity, SynchronousTime);
        }
        else
            break;

        if (run_succeed)
        {
            printf("1 -> 2 \n");
            TargetPosition = {0.0006,  0.0516,  1.1879,  -1.2394,  1.5702,   0};
            TargetVelocity = {0.0, 0.0, 0.0, 0.0, 0.0};
            run_succeed = tm_reflexxes::ReflexxesPositionRun(TM5, *IP_position, TargetPosition, TargetVelocity, SynchronousTime);
        }
        else
            break;

        if (run_succeed)
        {
            printf("1 -> 3 \n");
            TargetPosition = {0.6727, -0.0318, 1.6024, -1.5706, 0.8981, 0};
            TargetVelocity = {0.0, 0.0, 0.0, 0.0, 0.0};
            run_succeed = tm_reflexxes::ReflexxesPositionRun(TM5, *IP_position, TargetPosition, TargetVelocity, SynchronousTime);
        }
        else
            break;

        if (run_succeed)
        {
            printf("3 -> 4 \n");
            TargetPosition = {0.0006, 0.0509, 1.8490, -1.8999, 1.5702, 0};
            TargetVelocity = {0.0, 0.0, 0.0, 0.0, 0.0};
            run_succeed = tm_reflexxes::ReflexxesPositionRun(TM5, *IP_position, TargetPosition, TargetVelocity, SynchronousTime);
        }
        else
            break;
    }*/
}

int main(int argc, char **argv)
{

    double *x;
    double *y;
    double *z;
    double *qh = new double [60];
    x = new double [3000];
    y = new double [3000];
    z = new double [3000];

    /*const int STDIN = 0;
    int sockfd = -1;
    bool fgRun = false;
    std::string host;
    std::condition_variable data_cv;
    std::condition_variable data_cv_rt;*/

//**************************************************
//connect to mysql

    char save[100];
    MYSQL *con = mysql_init(NULL);
    if (con == NULL)
    {
        fprintf(stderr, "%s\n", mysql_error(con));
        exit(1);
    }
    if (mysql_real_connect(con, "140.113.149.61", "admin", "123", "nova", 0, NULL, 0) == NULL)
    {
        finish_with_error(con);
    }
    sprintf(save, "SELECT * FROM %s", "DATA621");
    string a = save;
    if (mysql_query(con, a.c_str()))
    {
        finish_with_error(con);
    }
    MYSQL_RES *result = mysql_store_result(con);
    if (result == NULL)
    {
        finish_with_error(con);
    }
    int num_fields = mysql_num_fields(result);
    MYSQL_ROW row;
    int record = 0;
    while ((row = mysql_fetch_row(result)))
    {

        x[record] = (atof(row[0]) / 1000.0) + CompenX; //0//0.1
        y[record] = (atof(row[1]) / 1000.0) + CompenY;//0//0.15
        z[record] = (atof(row[2]) / 1000.0) + CompenZ; //0.458//

        printf("x=%f  y=%f  z=%f\n", x[record], y[record], z[record]);
        record++;
    }
    mysql_free_result(result);
    mysql_close(con);

//**************************************************

    /*
        for (int i = 0; i < argc; i++)
        {
            printf("[DEBUG] arg%d:= %s\n", i, argv[i]);
        }
        host = argv[1];
        printf("[ INFO] host: %s", host.c_str());

        TmDriver TmRobot(data_cv, data_cv_rt, host, 0);

        char cstr[512];
        char delim[] = " ,;\t";
        char c;
    */
//*******************************
//homing
    double SynchronousTime = 0.5;
    std::vector<double> TargetVelocity(6), TargetPosition(6);
    RMLPositionInputParameters  *IP_position = new RMLPositionInputParameters(NUMBER_OF_DOFS);
    RMLVelocityInputParameters  *IP_vel      = new RMLVelocityInputParameters(NUMBER_OF_DOFS);

    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        IP_position->CurrentPositionVector->VecData[i] = 0.0;
        IP_position->CurrentVelocityVector->VecData[i] = 0.0;
        IP_position->CurrentAccelerationVector->VecData[i] = 0.0;
    }

    std::vector<double> Homing = { 0.24, -0.21, 0.36, 90.0 * DEG2RAD, 0.0 * DEG2RAD, 180.0 * DEG2RAD};
    TargetVelocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    if (GetQfromInverseKinematics(Homing, qh))
    {
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            TargetPosition[i] = qh[i];

        ReflexxesPositionRun_sim(*IP_position, TargetPosition, TargetVelocity, SynchronousTime);
        printf("Home reached\n");
    }
    else
        printf("Home out of limit \n");

//***************************
    int k = 0;
    double *qr = new double[60];
    double *qv = new double[60];
    Eigen::Matrix3d RotationMatrix;
    RotationMatrix << cos(PITCH * DEG2RAD), 0, sin(PITCH * DEG2RAD),
                   0, 1,                    0,
                   -sin(PITCH * DEG2RAD), 0, cos(PITCH * DEG2RAD);
    Eigen::Matrix3d IdentityMatrix;
    IdentityMatrix << 1, 0, 0,
                   0, 1, 0,
                   0, 0, 1;
    Eigen::Vector2d MinimumXZPoint;
    MinimumXZPoint << (0.26 + CompenX),
                   (0.0 + CompenY),
                   (-0.295 + CompenZ);
    std::vector<double> LastCarPosition;
    LastCarPosition.assign(3, 0.0f);

    std::vector<double> Velocitycalculate;
    Velocitycalculate.assign(6, 0.0f);

    while (k != record)
    {
        std::vector<double> CarPosition;
        CarPosition.assign(6, 0.0f);

        if (x[k] == -1)
            break;

        Eigen::Vector3d temp;
        temp << x[k], y[k], z[k];
        temp = RotationMatrix * temp;

        CarPosition[0] = temp(0) + ((I - RotationMatrix) * MinimumXZPoint)(0);
        CarPosition[1] = temp(1);
        CarPosition[2] = temp(2) + ((I - RotationMatrix) * MinimumXZPoint)(2);
        CarPosition[3] = 90.0 * DEG2RAD;
        CarPosition[4] = 0.0 * DEG2RAD;
        CarPosition[5] = (180.0 + PITCH) * DEG2RAD;

        for (int i = 0; i < 3; i++) {

            Velocitycalculate[i] = (CarPosition[i] - LastCarPosition[i]) / SynchronousTime;
        }
        


        if (GetQfromInverseKinematics(CarPosition, qr))
        {
            printf("%d   ", k );
            for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            {
                TargetPosition[i] = qr[i];
            }
            GetQdfromInverseJacobian(qr,Velocitycalculate,qv)
            //linear velocity = GetQdfromInverseJacobian*qr
            //target velocity = linear velocity
            for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            {
                TargetVelocity[i] = qv[i];
            }
            if (!ReflexxesPositionRun_sim(*IP_position, TargetPosition, TargetVelocity, SynchronousTime))
                break;

        }
        else
        {
            printf("inverse out of limit\n");
            print_info("Smooth Stop Activate...");
            *IP_vel->CurrentPositionVector     = *IP_position->CurrentPositionVector;
            *IP_vel->CurrentVelocityVector     = *IP_position->CurrentVelocityVector;
            *IP_vel->CurrentAccelerationVector = *IP_position->CurrentAccelerationVector;
            tm_reflexxes::ReflexxesSmoothStop_sim(*IP_vel, 0.5);
            break;
        }
        for (int i = 0; i < 3; i++) {
            LastCarPosition[i] = CarPosition[i];
        }

        k++;
    }

    printf("Finish drawing\n");
    print_info("Smooth Stop Activate...");
    *IP_vel->CurrentPositionVector     = *IP_position->CurrentPositionVector;
    *IP_vel->CurrentVelocityVector     = *IP_position->CurrentVelocityVector;
    *IP_vel->CurrentAccelerationVector = *IP_position->CurrentAccelerationVector;
    tm_reflexxes::ReflexxesSmoothStop_sim(*IP_vel, 0.5);

    delete [] qh;
    delete [] x;
    delete [] y;
    delete [] z;
    return 0;
}




