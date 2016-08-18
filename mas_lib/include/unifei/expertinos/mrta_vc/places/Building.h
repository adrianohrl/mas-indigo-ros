/**
 *  Building.h
 *
 *  Version: 1.2.2
 *  Created on: 02/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef PLACES_BUILDING_H_
#define PLACES_BUILDING_H_

#include "unifei/expertinos/mrta_vc/places/Campus.h"


namespace unifei 
{
	namespace expertinos
	{
		namespace mrta_vc
		{
			namespace places
			{
				class Building : public Place
				{

				public:
					Building(int id, std::string name, Campus campus, geometry_msgs::Polygon boundary, double x, double y, double theta = 0.0);
					Building(int id, std::string name, Campus campus, geometry_msgs::Polygon boundary, geometry_msgs::Pose pose_msg);
					Building(const mas_msgs::Place::ConstPtr& place_msg);
					Building(mas_msgs::Place place_msg);		
					virtual ~Building();

					Campus getCampus();
					mas_msgs::Place toMsg();
					void operator=(const Building& Building);

				protected:
					int getType();

				private:
					Campus campus_;

				};
			}
		}
	}
}		
					
#endif /* PLACES_BUILDING_H_ */
