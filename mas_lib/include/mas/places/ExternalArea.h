/**
 *  ExternalArea.h
 *
 *  Version: 1.2.4
 *  Created on: 02/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef PLACES_EXTERNAL_AREA_H_
#define PLACES_EXTERNAL_AREA_H_

#include "mas/places/Campus.h"


namespace mas 
{
	namespace places
	{
		class ExternalArea : public Place
		{

		public:
			ExternalArea(int id, std::string name, Campus campus, geometry_msgs::Polygon boundary, double x, double y, double theta = 0.0);
			ExternalArea(int id, std::string name, Campus campus, geometry_msgs::Polygon boundary, geometry_msgs::Pose pose_msg);
			ExternalArea(const mas_msgs::Place::ConstPtr& place_msg);
			ExternalArea(mas_msgs::Place place_msg);		
			virtual ~ExternalArea();

			Campus getCampus();
			mas_msgs::Place toMsg();
			void operator=(const ExternalArea& external_area);

		protected:
			int getType();

		private:
			Campus campus_;

		};
	}
}		
					
#endif /* PLACES_EXTERNAL_AREA_H_ */
