/**
 *  Building.h
 *
 *  Version: 1.0.0.0
 *  Created on: 02/04/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef BUILDING_H_
#define BUILDING_H_

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
					Building(int id, std::string name, Campus campus, geometry_msgs::Polygon boundary, double x, double y, double theta = 0);
					Building(int id, std::string name, Campus campus, geometry_msgs::Polygon boundary, geometry_msgs::Pose pose_msg);
					Building(const ::mrta_vc::Place::ConstPtr& place_msg);
					Building(::mrta_vc::Place place_msg);		
					~Building();

					Campus getCampus();
					::mrta_vc::Place toMsg();
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
					
#endif /* BUILDING_H_ */
