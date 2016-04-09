/**
 *  Office.h
 *
 *  Version: 1.0.0.0
 *  Created on: 02/04/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef OFFICE_H_
#define OFFICE_H_

#include "unifei/expertinos/mrta_vc/places/Floor.h"


namespace unifei 
{
	namespace expertinos
	{
		namespace mrta_vc
		{
			namespace places
			{
				class Office : public Place
				{

				public:
					Office(int id, std::string name, Floor floor, geometry_msgs::Polygon boundary, double x, double y, double theta = 0);
					Office(int id, std::string name, Floor floor, geometry_msgs::Polygon boundary, geometry_msgs::Pose pose_msg);
					Office(const ::mrta_vc::Place::ConstPtr& place_msg);
					Office(::mrta_vc::Place place_msg);		
					~Office();

					Floor getFloor();
					::mrta_vc::Place toMsg();
					void operator=(const Office& office);

				protected:
					int getType();

				private:
					Floor floor_;

				};
			}
		}
	}
}		
					
#endif /* OFFICE_H_ */
