/**
 *  Location.h
 *
 *  Version: 1.2.2
 *  Created on: 02/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef PLACES_LOCATION_H_
#define PLACES_LOCATION_H_

#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <mas_msgs/Location.h>

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
					Location(const mas_msgs::Location::ConstPtr& location_msg);
					Location(mas_msgs::Location location_msg);	
					virtual ~Location();

					double getX();
					double getY();
					double getTheta();
					geometry_msgs::Pose getPose();
					void setPose(double x, double y, double theta = 0.0);
					void setPose(geometry_msgs::Pose pose_msg);
					void setPose(mas_msgs::Location location_msg);
					void setPose(Location location);
					mas_msgs::Location toMsg();
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
					
#endif /* PLACES_LOCATION_H_ */
