uint8 type
uint32 id
string name				# When agent's type is Computer or Robot, this correspond to its hostname.
uint8 hierarchy_level
string login_name			# Only Voice Commanders use this parameter.
mrta_vc/Location location
#mrta_vc/Agent computer		# All Voice Commanders must be logged in a computer.
bool mobile				# This paremeter is not useful when agent's type is Person or Voice Commander, because they are always true.
bool holonomic				# This paremeter is only needed when agent's type is Robot.
geometry_msgs/Twist velocity	# Only robots have velocity.

# Types:
# -1 -> NON_AGENT
#  0 -> AGENT
#  1 -> PERSON
#  2 -> COMPUTER
#  3 -> ROBOT
#  4 -> VOICE_COMMANDER

# Hierarchy Levels:
#  0 -> VISITOR
#  1 -> INTERN
#  2 -> EMPLOYEE
#  3 -> SUPERVISOR
#  4 -> MIDDLE_MANAGER
#  5 -> SENIOR_MANAGER
#  6 -> CEO


