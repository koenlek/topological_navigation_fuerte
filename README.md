DISCLAIMER
==========
This is an updated version of the work presented here:

http://wiki.ros.org/topological_navigation

As described in this paper:

http://wiki.ros.org/Papers/ICRA2011_Konolige_Marder-Eppstein_Marthi

This stack was published for ROS Diamondback (it does not run on Fuerte, although the wiki does suggest that). As I, Koen Lekkerkerker, wanted to inspect this code as an inspiration for my own work, I decided to get it running on ROS Fuerte. This Git repository should contain all necessary files and instruction to get it running. Any issues can be reported on Github, although changes are small that I will respond. I have not been able to verify if it works completely as it should work. Especially the topological navigation graph (roadmap rviz topic) generation seems to create nodes at somewhat odd places. I was mostly unsure about some update that I had to make in laser_slam_mapper/src/mapper.cpp. I have not had any contact with the original authors of the code.
The code has not been ported to Groovy or Hydro yet, nor has it been catkinized, as that would require too much extra work. Please use rosbuild and Fuerte instead. Feel free to fork it and make it run on Hydro and/or catkinize it yourself.

INSTALLATION
============
To install/compile the Topological Navigation Stack:

First: we install some generic dependencies (maybe I forgot some, which will become apparent based on build errors):

- `sudo apt-get install ros-fuerte-navigation-tutorials ros-fuerte-pr2-controllers ros-fuerte-slam-karto ros-fuerte-warehousewg ros-fuerte-vision-opencv ros-fuerte-occupancy-grid-utils`

Second: we will build the topological_navigation stack and its dependencies in rosbuild:

- `rosws init ~/fuerte_ws /opt/ros/fuerte`
- `source ~/fuerte_ws/setup.bash` (and add it to your .bashrc for convenience)
- `roscd`
- `rosws set topological_navigation "https://github.com/koenlek/topological_navigation_fuerte/topological_navigation" --git`
- `rosws set vslam "https://github.com/koenlek/topological_navigation_fuerte/vslam" --git`
- `rosws set graph_mapping "https://github.com/koenlek/topological_navigation_fuerte/graph_mapping" --git`
- `rosws update topological_navigation`
- `rosws update vslam`
- `rosws update graph_mapping`
- `source ~/fuerte_ws/setup.bash`
- `rosdep check topological_navigation`
- `rosdep install topological_navigation` (I get a libtbb error, which I can ignore)
- `roscd topological_navigation`
- `rosmake`

If you get any errors during compilation, they are likely about missing dependencies. These should be installable through apt-get.
Another approach is to just retry compiling a few times. This has brougth me further in some exceptional cases.

USAGE
=====
	roslaunch topological_roadmap move_base_topo_stage.launch

- In case of such errors: bind() failed errno:98 Address already in use for socket: 0.0.0.0:27017
  run "sudo service mongodb stop" before roslaunching...
- You can use RVIZ to send 2D Nav Goals, for some reason, these should be quite close to the robots (which is visualized by its footprint): otherwise nothing will happen.
- You need to use the command line to send Topological Navigation Goals. See explanation below...
- Using the RVIZ select tool, you can select nodes from the roadmap, which can help you pick a goal to send to /move_base_topo/goal

RVIZ MARKERS
------------
Names of the markers aren't very logical...

- topological_map -> orange square. The frame of the current submap
- topological_map_edges -> red lines. Constraints between submaps.
- roadmap -> green lines, yellow dots. This is the actual navigable map! It is the topological roadmap that is used when planning based on a goal received on /move_base_topo/goal. Red line: planned route based on /move_base_topo/goal!
- constraint_graph -> blue lines, red dots. The constraint graph used for slam and I think the actual topological map is also based on this...
- graph_localization -> purple line. I think it shows a line between where SLAM thinks that the robot is and where it actually is? It seems that /move_base_node/local_costmap/robot_footprint draws the true footprint polygon.

SEND TOPOLOGICAL GOALS
----------------------
From command line, do:

	rostopic pub /move_base_topo/goal topological_nav_msgs/MoveBaseTopoActionGoal "header:
	  seq: 0
	  stamp:
	    secs: 0
	    nsecs: 0
	  frame_id: /map
	goal_id:
	  stamp:
	    secs: 0
	    nsecs: 0
	  id:
	goal:
	  goal_node: 1" 

Notes:

- goal_id:id:/grid1 -> you put the sub map containing the topo goal here. You can ommit this, or find this map using rostopic echo tricks
- goal:goal_node:1 -> put the topo roadmap node where you want to go here. You can find this node using RVIZ select
- Tip: you can use "select" in RVIZ to select nodes and figure out the number of this roadmap nodes!

ECHO TOPIC MESSAGES
-------------------

- Echo the navigable topological map (all nodes and edges, including costs of edges and the /gridXX map that nodes/edges belong to)

	`rostopic echo -n 1 /topological_roadmap`

- Get current submap. You can read it for exaple from:
	
	`rostopic echo -n 1 /topological_localization`

- Echo current goal (series of goals become visible if topo goal is sent):
	
	`rostopic echo /move_base_node/current_goal`

- Echo the set of submaps plus their constraints, i.e. relative positions:
	
	`rostopic echo -n 1 /topological_graph`

- Status throughout topo navigation:
	
	`rostopic echo /move_base_topo/status` outputs e.g:

		header: 
		  seq: 8156
		  stamp: 
		    secs: 1710
		    nsecs: 600000000
		  frame_id: ''
		status_list: 
		  - 
		    goal_id: 
		      stamp: 
		        secs: 1548
		        nsecs: 200000000
		      id: /grid44
		    status: 4
		    text: Failed to find a plan between node: 8 and node: 13

- Output of the topological plan: which nodes are going to be visited
	
	`rostopic echo /roadmap_path` gives such an output throughout a topo nav goal mission:
	
		path: [13, 11, 8, 10, 2, 1]
		---
		path: [11, 8, 10, 2, 1]
		---
		path: [11, 8, 10, 2, 1]
		---
		path: [11, 12, 8, 10, 2, 1]
		---
		path: [11, 12, 8, 10, 2, 1]
		---
		path: [8, 10, 2, 1]
		---
		path: [7, 10, 2, 1]
		---
		path: [2, 1]
		---

ADDITIONAL KNOWLEDGE 
=========
- Karto slam is used for scan-matching (according to article)
- For metric and topological planning Dijkstras algorithm is used...
- topological roadmap means -> navigable topological map
