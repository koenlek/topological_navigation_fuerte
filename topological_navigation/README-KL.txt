***** KNOWLEDGE *****
- Karto slam is used for scan-matching (according to article)
- For metric and topological planning Dijkstras algorithm is used...
- topological roadmap means -> navigable topological map

***** INSTALL/COMPILE *****
To compile/run Topological Navigation Stack:
- Use the code for graph_mapping and topological_navigation as in this fuerte workspace: also make sure that the vslam folder is in the workspace (you don't need to compile it). You need to use rosws to add the folders explicitely to your workspace, to make ros recognize it as packages/stacks. Do rosmake topological_mapping. Any errors should be related to missing dependencies: you should be able to install all through apt-get. 
- To run you need navigation_stage pkg, which is in the navigation_tutorials stack in the ros-fuerte-navigation-tutorials deb pkg.

***** USAGE *****
roslaunch topological_roadmap move_base_topo_stage.launch

- In case of such errors: bind() failed errno:98 Address already in use for socket: 0.0.0.0:27017
  run "sudo service mongodb stop" before roslaunching...
- You can use RVIZ to send 2D Nav Goals, for some reason, these should be quite close to the robots (which is visualized by its footprint): otherwise nothing will happen.
- You need to use the command line to send Topological Navigation Goals. See explanation below...
- Using the RVIZ select tool, you can select nodes from the roadmap, which can help you pick a goal to send to /move_base_topo/goal

* RVIZ MARKERS *
Names of the markers aren't very logical...

topological_map -> orange square. The frame of the current submap
topological_map_edges -> red lines. Constraints between submaps.
roadmap -> green lines, yellow dots. This is the actual navigable map! It is the topological roadmap that is used when planning based on a goal received on /move_base_topo/goal. Red line: planned route based on /move_base_topo/goal!
constraint_graph -> blue lines, red dots. The constraint graph used for slam and I think the actual topological map is also based on this...
graph_localization -> purple line. I think it shows a line between where SLAM thinks that the robot is and where it actually is? It seems that /move_base_node/local_costmap/robot_footprint draws the true footprint polygon.

* SEND TOPOLOGICAL GOALS *
Try this one for example:
Notes:
goal_id:id:/grid1 -> you put the sub map containing the topo goal here. You can ommit this, or find this map using rostopic echo tricks
goal:goal_node:1 -> put the topo roadmap node where you want to go here. You can find this node using RVIZ select

It turns out that you can ommit the goald_id:id:! 
Tip: you can use "select" in RVIZ to select nodes and figure out the number of this roadmap nodes!

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

OR

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
  id: /grid1
goal:
  goal_node: 1" 

* ECHO TOPIC MESSAGES *

- Echo the navigable topological map (all nodes and edges, including costs of edges and the /gridXX map that nodes/edges belong to)
rostopic echo -n 1 /topological_roadmap

- Get current submap
You can read it for exaple from:
rostopic echo -n 1 /topological_localization

- Echo current goal (series of goals become visible if topo goal is sent):
rostopic echo /move_base_node/current_goal

- Echo the set of submaps plus their constraints, i.e. relative positions:
rostopic echo -n 1 /topological_graph

- Status throughout topo navigation:
rostopic echo /move_base_topo/status
e.g.
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
rostopic echo /roadmap_path gives such an output throughout a topo nav goal mission:

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
