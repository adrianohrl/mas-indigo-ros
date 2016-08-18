/**
 *  Office.h
 *
 *  Version: 1.2.2
 *  Created on: 02/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef PLACES_OFFICE_H_
#define PLACES_OFFICE_H_

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
					Office(int id, std::string name, Floor floor, geometry_msgs::Polygon boundary, double x, double y, double theta = 0.0);
					Office(int id, std::string name, Floor floor, geometry_msgs::Polygon boundary, geometry_msgs::Pose pose_msg);
					Office(const mas_msgs::Place::ConstPtr& place_msg);
					Office(mas_msgs::Place place_msg);		
					virtual ~Office();

					Floor getFloor();
					mas_msgs::Place toMsg();
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
					
#endif /* PLACES_OFFICE_H_ */
