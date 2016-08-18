/**
 *  Floor.h
 *
 *  Version: 1.2.2
 *  Created on: 02/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef PLACES_FLOOR_H_
#define PLACES_FLOOR_H_

#include "unifei/expertinos/mrta_vc/places/Building.h"


namespace unifei 
{
	namespace expertinos
	{
		namespace mrta_vc
		{
			namespace places
			{
				class Floor : public Place
				{

				public:
					Floor(int id, std::string name, Building building, geometry_msgs::Polygon boundary, double x, double y, double theta = 0.0);
					Floor(int id, std::string name, Building building, geometry_msgs::Polygon boundary, geometry_msgs::Pose pose_msg);
					Floor(const mas_msgs::Place::ConstPtr& place_msg);
					Floor(mas_msgs::Place place_msg);		
					virtual ~Floor();

					Building getBuilding();
					mas_msgs::Place toMsg();
					void operator=(const Floor& floor);

				protected:
					int getType();

				private:
					Building building_;

				};
			}
		}
	}
}		
					
#endif /* PLACES_FLOOR_H_ */
