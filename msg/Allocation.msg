time allocation
mrta_vc/Task task
mrta_vc/Agent[] robots
time start
time end
uint8 state
int8 satisfaction

# States:
# 0 -> NOT_ALLOCATED
# 1 -> WAITING_ACCEPTATION
# 2 -> EXECUTING
# 3 -> ABORTED
# 4 -> FAILED
# 5 -> SUCCEEDED

# Satiscation Levels:
# -3 -> VERY_DISSATISFIED (or HORRIBLE)
# -2 -> DISSATISFIED (or VERY_BAD)
# -1 -> SOMEWHAT_DISSATISFIED (or BAD)
#  0 -> NEITHER_DISSATISFIED_NOR_SATISFIED (or FAIR_ENOUGH)
#  1 -> SOMEWHAT_SATISFIED (or GOOD)
#  2 -> SATISFIED (or VERY_GOOD)
#  3 -> VERY_SATISFIED (or EXCELLENT)


