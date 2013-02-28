auxtools
========

This project contains a ROS package with auxiliary tools:

  - calcSimLaser: listens to /map and simulates a 2D laser scan in 360º at each free cell
      - Output: distances.yml
      
  - myDrawer: listens to /pose /currentRoute and /particles to draw them on a /map with various options
  
  - routeSimulator: can be used to simulate route learning and reproduction with stage //TODO extend this description
