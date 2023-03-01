# Safe Autonomous Exploration

### Launch the environment
```bash
roslaunch autoexpl_simulations autoexpl_world.launch world:=docker
```

### Launch the slam 
```bash
roslaunch autoexpl_simulations autoexpl_slam.launch slam_methods:=karto
```

### Launch the autonomous exploration nodes
```bash
roslaunch autoexpl_ros autonomous_exploration.launch
```