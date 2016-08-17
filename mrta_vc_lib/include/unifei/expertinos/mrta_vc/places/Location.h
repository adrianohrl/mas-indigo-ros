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
					Location(double x = 0.0, double y = 0.0, double theta = 0.0);
					Location(geometry_msgs::Pose pose_msg);
					Location(const ::mrta_vc::Location::ConstPtr& location_msg);
					Location(::mrta_vc::Location location_msg);	
					virtual ~Location();

					double getX();
					double getY();
					double getTheta();
					geometry_msgs::Pose getPose();
					void setPose(double x, double y, double theta = 0.0);
					void setPose(geometry_msgs::Pose pose_msg);
					void setPose(::mrta_vc::Location location_msg);
					void setPose(Location location);
					::mrta_vc::Location toMsg();
					void operator=(const Location& location);

				private:
					double x_;
					double y_;
					double theta_;

				};
			}
		}
	}
}		
					
#endif /* LOCATION_H_ */
