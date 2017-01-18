/**
 *  This header file defines the Place class.
 *
 *  Version: 1.4.0
 *  Created on: 02/04/2016
 *  Modified on: 13/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _PLACES_PLACE_H_
#define _PLACES_PLACE_H_

#include <string>
#include <geometry_msgs/Polygon.h>
#include <mas_msgs/Place.h>
#include "utilities/exception.h"
#include "mas/places/location.h"

#define NON_PLACE -1
#define PLACE 0
#define CAMPUS 1
#define EXTERNAL_AREA 2
#define BUILDING 3
#define FLOOR 4
#define OFFICE 5
#define DESK 6

namespace mas 
{
	namespace places
	{
		class Place : public Location
		{

		public:
      Place(int id, std::string name, const geometry_msgs::Polygon& boundary, double x = 0.0, double y = 0.0, double theta = 0.0);
      Place(int id, std::string name, const geometry_msgs::Polygon& boundary, const geometry_msgs::Pose& pose_msg);
			Place(const mas_msgs::Place::ConstPtr& place_msg);
      Place(const mas_msgs::Place& place_msg);
			virtual ~Place();

      int getId() const;
      std::string getName() const;
      geometry_msgs::Polygon getBoundary() const;
			void setBoundary(geometry_msgs::Polygon boundary);
      virtual mas_msgs::Place to_msg() const;
      virtual void operator=(const Place& place);
      virtual bool operator==(const Place& place) const;
      bool operator!=(const Place& place) const;

		protected:
      virtual int getType() const;

		private:
			int id_;
			std::string name_;
			geometry_msgs::Polygon boundary_;

      bool isValidType(int type) const;

		};
	}
}		
					
#endif /* _PLACES_PLACE_H_ */
