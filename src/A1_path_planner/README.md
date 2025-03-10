# A1 Path Planner

- A1 path planner extends [nav2 theta star planner]

## Topic 

### publish
1. Topic
- /path_planning/destination 
2. Data type
- Data type std_msgs::msg::Uint8
3. Data 
- 0 - Original destination
- 1 - Alternative destination


## 

```
planner_server:
  ros__parameters:
    expected_planner_frequency: 5.0
    planner_patience: 5.0
    replanning: true
    use_sim_time: False
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "A1_path_planner/PathPlanner" # A1_path_planner::PathPlanner
      allow_unknown: false
      how_many_corners: 8                   # Dont change this parameter
      w_euc_cost: 1.0
      w_traversal_cost: 4.0
      w_heuristic_cost: 1.0

```
