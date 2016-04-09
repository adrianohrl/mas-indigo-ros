/**
 *  Floor.h
 *
 *  Version: 1.0.0.0
 *  Created on: 02/04/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef FLOOR_H_
#define FLOOR_H_

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
					Floor(int id, std::string name, Building building, geometry_msgs::Polygon boundary, double x, double y, double theta = 0);
					Floor(int id, std::string name, Building building, geometry_msgs::Polygon boundary, geometry_msgs::Pose pose_msg);
					Floor(const ::mrta_vc::Place::ConstPtr& place_msg);
					Floor(::mrta_vc::Place place_msg);		
					~Floor();

					Building getBuilding();
					::mrta_vc::Place toMsg();
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
					
#endif /* FLOOR_H_ */
