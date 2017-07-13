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
	    else
	        valid = true;

	    return valid;
	}

	bool CheckVelocityLimit(std::vector<double> qd)
	{
	    bool valid = true;

	    if(abs(qd[0]) > 180*DEG2RAD || abs(qd[1]) > 180*DEG2RAD || abs(qd[2]) > 180*DEG2RAD)
	    {
	        printf("[WARN] the 1th~3th joint : %10.4lf %10.4lf %10.4lf\n",qd[0],qd[3],qd[2] );
	        valid = false;
	    }
	    else if(abs(qd[3]) > 225*DEG2RAD || abs(qd[4]) > 225*DEG2RAD || abs(qd[5]) > 225*DEG2RAD)
	    {
	        printf("[WARN] the 4th~6th joint : %10.4lf %10.4lf %10.4lf\n",qd[3],qd[4],qd[5] );
	        valid = false;
	    }
	    else
	        valid = true;

	    return valid;
	}

	bool GetQfromInverseKinematics( std::vector<double> CartesianPosition, double *q_inv)
	{
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

	    Matrix2DoubleArray(T_,T);
	    printf(">>>> T \n");
	    printMatrix(T,4,16);

	    int num_sol =  tm_kinematics::inverse(T, q_inv);

	    delete [] T;
	    return CheckJointLimit(q_inv);
	}

	bool GetQdfromInverseJacobian(std::vector<double> CurrentPosition,std::vector<double> EFF_Velocity, std::vector<double>& qd)
	{

	    Eigen::Matrix<float, 6, 1> home,q;
	    home << 0, -PI*0.5, 0, PI*0.5, 0, 0;
	    Eigen::Matrix<float,6,1> effspd,jointspd;

	    home   << 0, -PI*0.5, 0, PI*0.5, 0, 0;
	    effspd << EFF_Velocity[0], EFF_Velocity[1], EFF_Velocity[2], EFF_Velocity[3], EFF_Velocity[4], EFF_Velocity[5];
	    q      << CurrentPosition[0], CurrentPosition[1], CurrentPosition[2], CurrentPosition[3], CurrentPosition[4], CurrentPosition[5];
	    q += home;

	    Eigen::Matrix<float, 6, 6> Inverse_Jacobian = tm_jacobian::Inverse_Jacobian(q);
	    jointspd = Inverse_Jacobian*effspd;
	    //cout << ">>>> Inverse jacobian" << endl;
	    //tm_jacobian::printMatrix(Inverse_Jacobian);

	    Matrix2DoubleVector(jointspd,qd);

	    return CheckVelocityLimit(qd);
	}

}