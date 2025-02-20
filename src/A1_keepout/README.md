airbot_navigation

param 

global_costmap
        keepout_zone:
          topic: /A1_keepout/point_cloud
          sensor_frame: "map"
          clearing: True
          marking: True
          data_type: "PointCloud2"
          obstacle_range: 10.0
          observation_persistence: 100.0


