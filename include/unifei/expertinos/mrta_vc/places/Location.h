/**
 *  Location.h
 *
 *  Version: 1.0.0.0
 *  Created on: 02/04/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef LOCATION_H_
#define LOCATION_H_

#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include "mrta_vc/Location.h"

namespace unifei 
{
	namespace expertinos
	{
		namespace mrta_vc
		{
			namespace places
			{
				class Location 
				{

				public:
					Location(double x, double y, double theta);	
					Location(geometry_msgs::Pose pose_msg);
					Location(const ::mrta_vc::Location::ConstPtr& location_msg);
					Location(::mrta_vc::Location location_msg);	
					~Location();

					double getX();
					double getY();
					double getTheta();
					geometry_msgs::Pose getPose();
					void setPose(double x, double y, double theta);
					void setPose(geometry_msgs::Pose pose_msg);
					void setPose(::mrta_vc::Location location_msg);
					::mrta_vc::Location toMsg();

				private:
					double x_, y_, theta_;

				};
			}
		}
	}
}		
					
#endif /* LOCATION_H_ */
