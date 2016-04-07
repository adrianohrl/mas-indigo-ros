/**
 *  Place.h
 *
 *  Version: 1.0.0.0
 *  Created on: 02/04/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef PLACE_H_
#define PLACE_H_

#include <string>
#include <geometry_msgs/Polygon.h>
#include "mrta_vc/Place.h"
#include "unifei/expertinos/mrta_vc/places/Location.h"

#define NON_PLACE -1
#define PLACE 0
#define CAMPUS 1
#define EXTERNAL_AREA 2
#define BUILDING 3
#define FLOOR 4
#define OFFICE 5
#define DESK 6

namespace unifei 
{
	namespace expertinos
	{
		namespace mrta_vc
		{
			namespace places
			{
				class Place : public Location
				{

				public:
					Place(int id, std::string name, geometry_msgs::Polygon boundary, double x = 0, double y = 0, double theta = 0);
					Place(int id, std::string name, geometry_msgs::Polygon boundary, geometry_msgs::Pose pose_msg);
					Place(const ::mrta_vc::Place::ConstPtr& place_msg);
					Place(::mrta_vc::Place place_msg);		
					~Place();

					int getId();
					std::string getName();
					geometry_msgs::Polygon getBoundary();
					void setBoundary(geometry_msgs::Polygon boundary);
					::mrta_vc::Place toMsg();
					bool equals(Place place);
					bool operator==(const Place& place);
					bool operator!=(const Place& place);
					void operator=(const Place& place);

				protected:
					int getType();

				private:
					int id_;
					std::string name_;
					geometry_msgs::Polygon boundary_;

					bool isValidType(int type);

				};
			}
		}
	}
}		
					
#endif /* PLACE_H_ */
