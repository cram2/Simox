#include <string>


static const std::string AABB_center_extents =
    R"(
{
  "center": [
    45.0,
    90.0,
    135.0
  ],
  "extents": [
    110.0,
    220.0,
    330.0
  ]
}
)";


static const std::string AABB_min_max =
    R"(
{
  "min": [
    -10.0,
    -20.0,
    -30.0
  ],
  "max": [
    100.0,
    200.0,
    300.0
  ]
}
)";


static const std::string OrientedBox_position_orientation_extents =
    R"(
{
  "extents": [
    100.0,
    200.0,
    300.0
  ],
  "orientation": {
    "qw": 0.707,
    "qx": 0.0,
    "qy": 0.707,
    "qz": 0.0
  },
  "position": [
    10.0,
    -20.0,
    0.0
  ]
}
)";


static const std::string OrientedBox_pos_ori_dimensions =
    R"(
{
  "dimensions": [
    100.0,
    200.0,
    300.0
  ],
  "ori": {
    "qw": 0.707,
    "qx": 0.0,
    "qy": 0.707,
    "qz": 0.0
  },
  "pos": [
    10.0,
    -20.0,
    0.0
  ]
}
)";
