uint8 type
uint32 id
string name
mrta_vc/Location location
geometry_msgs/Polygon boundary
#mrta_vc/Place parent	# places whose type is Campus don't have a parent place

# Types:
# 0 -> NON_PLACE
# 1 -> PLACE
# 2 -> CAMPUS
# 3 -> EXTERNAL_AREA
# 4 -> BUILDING
# 5 -> OFFICE
# 6 -> DESK

