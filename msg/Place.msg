uint8 type
uint32 id
string name
mrta_vc/Location location
geometry_msgs/Polygon boundary
#mrta_vc/Place parent	# places whose type is Campus don't have a parent place
string floor		# only places whose type is Office belongs to a floor

# Types:
# 0 -> Campus
# 1 -> External Areas
# 2 -> Building
# 3 -> Office
# 4 -> Desk

