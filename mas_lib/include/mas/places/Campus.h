/**
 *  Campus.h
 *
 *  Version: 1.2.4
 *  Created on: 02/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef PLACES_CAMPUS_H_
#define PLACES_CAMPUS_H_

#include "mas/places/Place.h"


namespace mas 
{
	namespace places
	{
		class Campus : public Place
		{

		public:	
			Campus(int id, std::string name, geometry_msgs::Polygon boundary, double x, double y, double theta = 0.0);
			Campus(int id, std::string name, geometry_msgs::Polygon boundary, geometry_msgs::Pose pose_msg);
			Campus(const mas_msgs::Place::ConstPtr& place_msg);
			Campus(mas_msgs::Place campus_msg);
			virtual ~Campus();
			
		protected:
			int getType();

		};
	}
}		
					
#endif /* PLACES_CAMPUS_H_ */
