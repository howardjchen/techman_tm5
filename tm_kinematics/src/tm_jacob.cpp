/*********************************************************************
 *                      Apache License
 *                 Version 2.0, January 2004
 *               http://www.apache.org/licenses/
 *
 * tm_jacob.cpp
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
/*
 *  tm_jacob.cpp
 *
 *  Created on: June 13, 2017
 *      Author: Howard Chen
 */
#include "tm_kinematics/tm_kin.h"


namespace tm_jacobian {

	Eigen::Matrix<float, 6, 6> Forward_Jacobian(Eigen::Matrix<float, 6,1> q)
	{
		Eigen::Matrix<float, 6, 6> jacobian = Eigen::Matrix<float, 6, 6>::Zero();
		jacobian << D6*(cos(q(0))*cos(q(4)) + sin(q(4))*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))))) - D4*cos(q(0)) - D5*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0)))) - A2*cos(q(1))*sin(q(0)) - A3*cos(q(1))*cos(q(2))*sin(q(0)) + A3*sin(q(0))*sin(q(1))*sin(q(2)),   D5*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1)))) - A2*cos(q(0))*sin(q(1)) - D6*sin(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2)))) - A3*cos(q(0))*cos(q(1))*sin(q(2)) - A3*cos(q(0))*cos(q(2))*sin(q(1)),   D5*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1)))) - D6*sin(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2)))) - A3*cos(q(0))*cos(q(1))*sin(q(2)) - A3*cos(q(0))*cos(q(2))*sin(q(1)),   D5*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1)))) - D6*sin(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2)))), -D6*(sin(q(0))*sin(q(4)) - cos(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))))),                                                                                                                                       				        						   0.,
	 				D5*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2)))) - D4*sin(q(0)) + D6*(cos(q(4))*sin(q(0)) + sin(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))))) + A2*cos(q(0))*cos(q(1)) + A3*cos(q(0))*cos(q(1))*cos(q(2)) - A3*cos(q(0))*sin(q(1))*sin(q(2)), - D5*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1)))) - D6*sin(q(4))*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0)))) - A2*sin(q(0))*sin(q(1)) - A3*cos(q(1))*sin(q(0))*sin(q(2)) - A3*cos(q(2))*sin(q(0))*sin(q(1)), - D5*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1)))) - D6*sin(q(4))*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0)))) - A3*cos(q(1))*sin(q(0))*sin(q(2)) - A3*cos(q(2))*sin(q(0))*sin(q(1)), - D5*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1)))) - D6*sin(q(4))*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0)))),  D6*(cos(q(0))*sin(q(4)) - cos(q(4))*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))))),                                                                                                                                                      								   0.,
																					                                                                                                                                                                                                                                                                                                                                                                                       				 0.,                                                                                             					 A3*sin(q(1))*sin(q(2)) - D5*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2)))) - A3*cos(q(1))*cos(q(2)) - A2*cos(q(1)) - D6*sin(q(4))*(cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1)))),                                                                                   					   A3*sin(q(1))*sin(q(2)) - A3*cos(q(1))*cos(q(2)) - D5*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2)))) - D6*sin(q(4))*(cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1)))),                                                                 				- D5*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2)))) - D6*sin(q(4))*(cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1)))),                                                     			 -D6*cos(q(4))*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2)))),                                                                                                                                                     								   0.,
																					                     	                                                                                                                                                                                                                                                                                                                                                                  			 0.,                                                                                                                                                                                                                                                                                                                                                  																		 -sin(q(0)),                                                                                                                                                                                                                                                                                                                            																	-sin(q(0)),                                                                                                                                                                                                                                                                  														 -sin(q(0)),                                   		 cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))),   cos(q(4))*sin(q(0)) + sin(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1)))),
																					                                                                                                                                                                                                                                                                                                                                                                                       				 0.,                                                                                                                                                                                                                                                                                                                                                   																		  cos(q(0)),                                                                                                                                                                                                                                                                                                                             																	 cos(q(0)),                                                                                                                                                                                                                                                                   														  cos(q(0)),                                   		 cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))), - cos(q(0))*cos(q(4)) - sin(q(4))*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1)))),
																					                                                                                                                                                                                                                                                                                                                                                                                       				 1.,                                                                                                                                                                                                                                                                                                                                                        																		 0.,                                                                                                                                                                                                                                                                                                                                  																		0.,                                                                                                                                                                                                                                                                        															 0.,                                                                   				 cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))),                                                    			   -sin(q(4))*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))));
		return jacobian;                  
	}

	Eigen::Matrix<float, 6, 6> Inverse_Jacobian(Eigen::Matrix<float, 6,1> q)
	{
		Eigen::Matrix<float, 6, 6> jacobian = Eigen::Matrix<float, 6, 6>::Zero();
		jacobian << D6*(cos(q(0))*cos(q(4)) + sin(q(4))*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))))) - D4*cos(q(0)) - D5*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0)))) - A2*cos(q(1))*sin(q(0)) - A3*cos(q(1))*cos(q(2))*sin(q(0)) + A3*sin(q(0))*sin(q(1))*sin(q(2)),   D5*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1)))) - A2*cos(q(0))*sin(q(1)) - D6*sin(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2)))) - A3*cos(q(0))*cos(q(1))*sin(q(2)) - A3*cos(q(0))*cos(q(2))*sin(q(1)),   D5*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1)))) - D6*sin(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2)))) - A3*cos(q(0))*cos(q(1))*sin(q(2)) - A3*cos(q(0))*cos(q(2))*sin(q(1)),   D5*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1)))) - D6*sin(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2)))), -D6*(sin(q(0))*sin(q(4)) - cos(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))))),                                                                                                                                       				        						   0.,
	 				D5*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2)))) - D4*sin(q(0)) + D6*(cos(q(4))*sin(q(0)) + sin(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))))) + A2*cos(q(0))*cos(q(1)) + A3*cos(q(0))*cos(q(1))*cos(q(2)) - A3*cos(q(0))*sin(q(1))*sin(q(2)), - D5*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1)))) - D6*sin(q(4))*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0)))) - A2*sin(q(0))*sin(q(1)) - A3*cos(q(1))*sin(q(0))*sin(q(2)) - A3*cos(q(2))*sin(q(0))*sin(q(1)), - D5*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1)))) - D6*sin(q(4))*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0)))) - A3*cos(q(1))*sin(q(0))*sin(q(2)) - A3*cos(q(2))*sin(q(0))*sin(q(1)), - D5*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1)))) - D6*sin(q(4))*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0)))),  D6*(cos(q(0))*sin(q(4)) - cos(q(4))*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))))),                                                                                                                                                      								   0.,
																					                                                                                                                                                                                                                                                                                                                                                                                       				 0.,                                                                                             					 A3*sin(q(1))*sin(q(2)) - D5*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2)))) - A3*cos(q(1))*cos(q(2)) - A2*cos(q(1)) - D6*sin(q(4))*(cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1)))),                                                                                   					   A3*sin(q(1))*sin(q(2)) - A3*cos(q(1))*cos(q(2)) - D5*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2)))) - D6*sin(q(4))*(cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1)))),                                                                 				- D5*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2)))) - D6*sin(q(4))*(cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1)))),                                                     			 -D6*cos(q(4))*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2)))),                                                                                                                                                     								   0.,
																					                     	                                                                                                                                                                                                                                                                                                                                                                  			 0.,                                                                                                                                                                                                                                                                                                                                                  																		 -sin(q(0)),                                                                                                                                                                                                                                                                                                                            																	-sin(q(0)),                                                                                                                                                                                                                                                                  														 -sin(q(0)),                                   		 cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))),   cos(q(4))*sin(q(0)) + sin(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1)))),
																					                                                                                                                                                                                                                                                                                                                                                                                       				 0.,                                                                                                                                                                                                                                                                                                                                                   																		  cos(q(0)),                                                                                                                                                                                                                                                                                                                             																	 cos(q(0)),                                                                                                                                                                                                                                                                   														  cos(q(0)),                                   		 cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))), - cos(q(0))*cos(q(4)) - sin(q(4))*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1)))),
																					                                                                                                                                                                                                                                                                                                                                                                                       				 1.,                                                                                                                                                                                                                                                                                                                                                        																		 0.,                                                                                                                                                                                                                                                                                                                                  																		0.,                                                                                                                                                                                                                                                                        															 0.,                                                                   				 cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))),                                                    			   -sin(q(4))*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))));
		return jacobian.inverse();                  
	}


	void printMatrix(Eigen::MatrixXf InputMatrix)
	{
		Eigen::MatrixXf InputTranspose = InputMatrix.transpose();
		short count = 0;
		int row = InputMatrix.rows();
		int col = InputMatrix.cols();

		for (int i = 0; i < row*col; ++i)
		{
			printf("%10.4f ", InputTranspose(i));
			if (count == row-1)
			{
				count = 0;
				printf("\n");
			}
			else
				count++;
		}
	}

	void printVector(Eigen::VectorXd InputVector)
	{
		Eigen::VectorXd InputTranspose = InputVector.transpose();
		int row = InputVector.rows();
		int col = InputVector.cols();

		for (int i = 0; i < row*col; ++i)
			printf("%10.4f ", InputTranspose(i));
		printf("\n");
	}

	void printMatrix(double *InputMatrix, short col, int num)
	{
		short count = 0;
		if(num == 0)
			num = 1*col;
		
		for (int i = 0; i < num; ++i)
		{
			printf("%10.4f ", InputMatrix[i]);
			if (count == col-1)
			{
				count = 0;
				printf("\n");
			}
			else
				count++;
		}
		printf("\n");
	}

	void printVector(std::vector<double> vec)
	{
		for (int i = 0; i < vec.size(); ++i)
		{
			printf("%10.4f ", vec[i]);

		}
		printf("\n");
	}

	void Matrix2DoubleArray(Eigen::MatrixXf InputMatrix, double *T)
	{
		Eigen::MatrixXf InputTranspose = InputMatrix.transpose();
		short row = InputMatrix.rows();
		short col = InputMatrix.cols();

		for (int i = 0; i < row*col; ++i)
			T[i] = InputTranspose(i);
	}

	void Matrix2DoubleVector(Eigen::MatrixXf InputMatrix, std::vector<double> &vec)
	{
		Eigen::MatrixXf InputTranspose = InputMatrix.transpose();
		short row = InputMatrix.rows();
		short col = InputMatrix.cols();

		for (int i = 0; i < row*col; ++i)
			vec[i] = InputTranspose(i);
	}

}