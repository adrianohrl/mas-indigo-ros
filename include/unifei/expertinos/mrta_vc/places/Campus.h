/**
 *  Campus.h
 *
 *  Version: 1.0.0.0
 *  Created on: 02/04/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef CAMPUS_H_
#define CAMPUS_H_

#include "unifei/expertinos/mrta_vc/places/Place.h"


namespace unifei 
{
	namespace expertinos
	{
		namespace mrta_vc
		{
			namespace places
			{
				class Campus : public Place
				{

				public:	
					Campus(int id, std::string name, geometry_msgs::Polygon boundary, double x, double y, double theta = 0);
					Campus(int id, std::string name, geometry_msgs::Polygon boundary, geometry_msgs::Pose pose_msg);
					Campus(const ::mrta_vc::Place::ConstPtr& place_msg);
					Campus(::mrta_vc::Place campus_msg);
					~Campus();
					
				protected:
					int getType();

				};
			}
		}
	}
}		
					
#endif /* CAMPUS_H_ */
