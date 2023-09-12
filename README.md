# proximitygrid
The package implements the proximity grid class. The proximity grid is a polar grid with different sectors from *angle_min* to *angle_max* with an increment of *angle_inc* degrees. Each sector has a value representing the current distance to an object in the surroundings frome a minimum range of *range_min* to *range_max*.

The package provides different nodes to convert the standard sensor messages into a proximity grid. It also provides a custom message for the grid: *proximity_grid/ProximityGrid*

### LaserScanToProximityGrid
The node converts the sensors_msgs/LaserScan into a ProximityGrid.

#### Subscribed topic:
- /hokuyo/scan [sensors_msgs/LaserScan]

#### Published topic:
- /proximity_grid [proximity_grid/ProximityGrid]

#### Parameters:
- angle_min [float] (default: -120.0 degrees) 
- angle_max [float] (default: 120.0 degrees)
- angle_inc [float] (default: 9.0 degrees)
- range_min [float] (default: 0.0 m)
- range_max [float] (default: 6.0 m)

### PointToProximityGrid
The node convers a geometry_msgs::PointStamped into a ProximityGrid.

#### Subscribed topic:
- /bci_command [geometry_msgs/PointStamped]

#### Published topic:
- /proximity_grid [proximity_grid/ProximityGrid]

#### Parameters:
Same as before

