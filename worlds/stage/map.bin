# map.inc - useful setup for a floorplan bitmap
# Authors: Hugo Costelha based on Richard Vaughan work
# $Id$

define floorplan model
(
  # sombre, sensible, artistic
  color "gray30"

  # Do not create a wall around the map
  boundary 0

  trail_length 10000 # Number of trail markers to store

  gui_nose 0 # Do not show heading
  gui_grid 0 # Do not show a finner grid
  gui_move 0 # Do not allow the environment to be moved
  gui_outline 0 # Do not draw bounding box
  gripper_return 0 # Do not allow a gripper to grab the environment
  fiducial_return 0 # Not detectable as markers
  ranger_return 1 # Detectable by range sensors
  obstacle_return 1 # Not detectable as an obstacle (use the range sensors)
)

# This zone is useful to show a specific area of the environment
define zone model
(
  color	"orange"
  size [ 4 4 0.02 ]

  gui_nose 0
  gui_grid 0
  gui_move 1 # Can be moved
  gui_outline 0

  # insensible to collision and range sensors
  obstacle_return 0
  ranger_return -1  # transparent to range sensors
)

# This zone is useful to show a specific area of the environment
define obstacle model
(
  color "orange"
  size [ 0.7 0.7 0.8 ]

  gui_nose 0
  gui_grid 0
  gui_move 0 # Cannot be moved
  gui_outline 0

  # Sensible to collision and range sensors
  obstacle_return 1
  ranger_return 1
)
