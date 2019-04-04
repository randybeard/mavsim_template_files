# msg_map
#   - message type for map of world
#
#  mavsim_python
#      - Beard & McLain, PUP, 2012
#      - Update history:
#          4/3/2019 - BGM
import numpy as np

class msg_map:
    def __init__(self, PLAN):
            # flag to indicate if the map has changed
            self.flag_map_changed = 0
            # the city is of size (width)x(width)
            self.city_width = PLAN.city_width
            # number of blocks in city
            self.num_city_blocks = PLAN.num_blocks
            # percent of block that is street.
            self.street_width = PLAN.city_width/PLAN.num_blocks*PLAN.street_width
            # maximum height of buildings
            self.building_max_height = PLAN.building_height
            # an array of building heights
            self.building_height = PLAN.building_height * np.random.rand(PLAN.num_blocks, PLAN.num_blocks)
            # the width of the buildings (all the same)
            self.building_width = PLAN.city_width/PLAN.num_blocks * (1-PLAN.street_width)
            # an array of the north corner of buildings
            self.building_north = np.zeros(PLAN.num_blocks)
            for i in range(0, PLAN.num_blocks):
                self.building_north[i] = 0.5*PLAN.city_width/PLAN.num_blocks*(2*(i)+1)
            # an array of the east corner of buildings
            self.building_east = self.building_north
