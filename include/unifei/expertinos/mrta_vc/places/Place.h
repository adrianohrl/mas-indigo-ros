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

#define NONE -1
#define CAMPUS 0
#define EXTERNAL_AREA 1
#define BUILDING 2
#define FLOOR 3
#define OFFICE 4
#define DESK 5

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
					Place(int id, std::string name, geometry_msgs::Polygon boundary, double x, double y, double theta = 0);
					Place(int id, std::string name, geometry_msgs::Polygon boundary, geometry_msgs::Pose pose_msg);
					Place(const ::mrta_vc::Place::ConstPtr& place_msg);
					Place(::mrta_vc::Place place_msg);		
					~Place();

					int getId();
					std::string getName();
					geometry_msgs::Polygon getBoundary();
					void setBoundary(geometry_msgs::Polygon boundary);
					virtual ::mrta_vc::Place toMsg();
					bool operator==(const Place& place);
					bool operator!=(const Place& place);

				protected:
					virtual int getType() = 0;

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
