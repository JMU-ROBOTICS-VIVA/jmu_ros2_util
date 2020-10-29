"""Map class that can generate and read OccupancyGrid messages.

Grid cell entries are occupancy probababilities represented as
integers in the range 0-100. Value of -1 indicates unknown.

Author: Nathan Sprague
Version: 10/29/2020

"""

import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Time


class Map:
    """ The Map class represents an occupancy grid.

    Map entries are stored as 8-bit integers.

    Public instance variables:

        width      --  Number of columns in the occupancy grid.
        height     --  Number of rows in the occupancy grid.
        resolution --  Width of each grid square in meters.
        origin_x   --  Position of the grid cell (0,0) in
        origin_y   --    in the map coordinate system.
        grid       --  numpy array with height rows and width columns.

    Note that x increases with increasing column number and y increases
    with increasing row number.
    """

    def __init__(self, *args, **kwargs):
        """ Construct an empty occupancy grid.

        Can be called -either- with a single OccupancyGrid message as
        the argument, or with any of the following provided as named
        arguments:

           keyword arguments:
                   origin_x,
                   origin_y  -- The position of grid cell (0,0) in the
                                map coordinate frame. (default -2.5, -2.5)
                   resolution-- width and height of the grid cells
                                in meters. (default .1)
                   width,
                   height    -- The grid will have height rows and width
                                columns cells.  width is the size of
                                the x-dimension and height is the size
                                of the y-dimension. (default 50, 50)

         The default arguments put (0,0) in the center of the grid.
        """

        if len(args) == 1 and isinstance(args[0], OccupancyGrid):
            self._init_from_message(args[0])
        elif len(args) == 0:
            self._init_empty(kwargs)
        else:
            raise ValueError("Constructor only supports named arguments.")

    def _init_empty(self, kwargs):
        """ Set up an empty map using keyword arguments. """
        self.frame_id = "map"
        self.stamp = Time()  # There is no great way to get the actual time.
        self.origin_x = kwargs.get('origin_x', -2.5)
        self.origin_y = kwargs.get('origin_y', -2.5)
        self.width = kwargs.get('width', 50)
        self.height = kwargs.get('height', 50)
        self.resolution = kwargs.get('resolution', .1)
        self.grid = np.zeros((self.height, self.width))

    def _init_from_message(self, map_message):
        """
        Set up a map as an in-memory version of an OccupancyGrid message
        """
        self.frame_id = map_message.header.frame_id
        self.stamp = map_message.header.stamp
        self.width = map_message.info.width
        self.height = map_message.info.height
        self.resolution = map_message.info.resolution
        self.origin_x = map_message.info.origin.position.x
        self.origin_y = map_message.info.origin.position.y
        self.grid = np.array(map_message.data, dtype='int8').reshape(self.height, self.width)

    def to_msg(self):
        """ Return a nav_msgs/OccupancyGrid representation of this map. """
        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = self.frame_id
        grid_msg.header.stamp = self.stamp
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height
        grid_msg.info.origin = Pose()
        grid_msg.info.origin.position.x = self.origin_x
        grid_msg.info.origin.position.y = self.origin_y
        grid_msg.info.origin.orientation.x = 0.0
        grid_msg.info.origin.orientation.y = 0.0
        grid_msg.info.origin.orientation.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0

        grid_msg.data = [int(val) for val in self.grid.flatten()]
        return grid_msg

    def cell_position(self, row, col):
        """
        Determine the x, y coordinates of the center of a particular grid cell.
        """
        x = col * self.resolution + .5 * self.resolution + self.origin_x
        y = row * self.resolution + .5 * self.resolution + self.origin_y
        return x, y

    def cell_index(self, x, y):
        """
        Helper method for finding map index.  x and y are in the map
        coordinate system.
        """
        x -= self.origin_x
        y -= self.origin_y
        row = int(np.floor(y / self.resolution))
        col = int(np.floor(x / self.resolution))
        return row, col

    def set_cell(self, x, y, val):
        """
        Set the value in the grid cell containing position (x,y).
        x and y are in the map coordinate system.  No effect if (x,y) 
        is out of bounds.
        """
        row, col = self.cell_index(x, y)
        try:
            if row >= 0 and col >= 0:
                self.grid[row, col] = val
        except IndexError:
            pass

    def get_cell(self, x, y):
        """
        Get the value from the grid cell containing position (x,y).
        x and y are in the map coordinate system.  Return 'nan' if
        (x,y) is out of bounds.
        """
        row, col = self.cell_index(x, y)
        try:
            if row >= 0 and col >= 0:
                return self.grid[row, col]
            else:
                return float('nan')
        except IndexError:
            return float('nan')
