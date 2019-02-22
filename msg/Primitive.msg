uint8 PLANE = 1
uint8 CYLINDER = 3

Header header
float64[] coef
float64[3] normal
geometry_msgs/Point center
geometry_msgs/Vector3 min
geometry_msgs/Vector3 max
geometry_msgs/Point32[] polygon
std_msgs/Int32 size
sensor_msgs/PointCloud2 cloud
bool is_vertical
int32 type
geometry_msgs/Point object_center
geometry_msgs/Point32[] object_polygon
