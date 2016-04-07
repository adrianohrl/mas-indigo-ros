/**
 *  Robot.h
 *
 *  Version: 1.0.0.0
 *  Created on: 05/04/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include <geometry_msgs/Twist.h>
#include "unifei/expertinos/mrta_vc/agents/Computer.h"

namespace unifei 
{
	namespace expertinos
	{
		namespace mrta_vc
		{
			namespace agents
			{
				class Robot : public Computer
				{

				public:
					Robot(int id, std::string hostname, bool holonomic, bool mobile = true, double x = 0, double y = 0, double theta = 0);
					Robot(int id, std::string hostname, bool holonomic, bool mobile, geometry_msgs::Pose pose_msg);
					Robot(const ::mrta_vc::Agent::ConstPtr& robot_msg);
					Robot(::mrta_vc::Agent robot_msg);		
					~Robot();

					bool isHolonomic();
					double getVelX();
					double getVelY();
					double getVelTheta();
					geometry_msgs::Twist getVelocity();
					void setVelocity(double x = 0, double y = 0, double theta = 0);
					void setVelocity(geometry_msgs::Twist twist_msg);
					::mrta_vc::Agent toMsg();
					void operator=(const Robot& Robot);
					
				protected:
					int getType();

				private:
					bool holonomic_;
					double vel_x_;
					double vel_y_;
					double vel_theta_;

				};
			}
		}
	}
}		
					
#endif /* ROBOT_H_ */
