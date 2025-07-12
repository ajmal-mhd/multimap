# TurtleBot3 Multi-Map Navigator

## Setup PostgreSQL Database

### 1. Start PostgreSQL container

```bash
sudo docker run --name map_db \
  -e POSTGRES_PASSWORD=password \
  -e POSTGRES_DB=map_db \
  -p 5432:5432 \
  -d postgres
```

### 2. Connect to database and create tables

```bash
sudo docker exec -it map_db psql -U postgres -d map_db
```

### 3. Create the multimap table

```sql
CREATE TABLE multimap (
  from_room   VARCHAR(50),
  to_room     VARCHAR(50),
  from_pose_x DOUBLE PRECISION,
  from_pose_y DOUBLE PRECISION,
  from_yaw    DOUBLE PRECISION,
  to_pose_x   DOUBLE PRECISION,
  to_pose_y   DOUBLE PRECISION,
  to_yaw      DOUBLE PRECISION
);
```

### 4. Insert wormhole data

```sql
INSERT INTO multimap (from_room, to_room, from_pose_x, from_pose_y, from_yaw, to_pose_x, to_pose_y, to_yaw) VALUES
('room1', 'room2', -0.7, -8.25, -1.57, 0.9, 0.0, 0.0),
('room1', 'room4', 0.0, 0.0, 3.14, 0.3, 0.0, 0.0),
('room2', 'room3', 5.20, -0.25, -1.57, 0.5, 0.0, 0),
('room2', 'room1', 0.9, 0.0, 3.14, -0.7, -8.25, 1.57),
('room3', 'room2', 0.5, 0.0, 3.14, 5.20, -0.25, 1.57),
('room4', 'room1', 0.3, 0.0, 3.14, 0.0, 0.0, 0.0);
```

To exit the psql terminal:
```sql
\q
```


## Build and Run the Package

### 1. Build the workspace

```bash
cd turtlebot3_ws/
colcon build
```

### 2. Environment setup

Open 4 terminals and run the following commands in each:

```bash
cd turtlebot3_ws/
source install/setup.bash 
export TURTLEBOT3_MODEL=burger
```

### 3. Launch services

**Terminal 1:** Launch Gazebo simulation
```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

**Terminal 2:** Launch navigation stack
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$PWD/maps/room1.yaml
```

**Terminal 3:** Run multi-map action server
```bash
ros2 run multimap multi_map_navigator
```

### 4. Navigation commands

> **Note:** Refer to `room_layout.jpg` for room names and wormhole locations.

**Terminal 4:** Send navigation goals

Navigate to room 2:
```bash
ros2 action send_goal navigate_to_waypoint multimap/action/NavigateToWaypoint "{target_map: 'room2', waypoint: [1.7,2.25,1.57]}"
```

Navigate to room 3:
```bash
ros2 action send_goal navigate_to_waypoint multimap/action/NavigateToWaypoint "{target_map: 'room3', waypoint: [2.0,0.0,0.0]}"
```

Navigate to room 4:
```bash
ros2 action send_goal navigate_to_waypoint multimap/action/NavigateToWaypoint "{target_map: 'room4', waypoint: [1.7,0.0,0.0]}"
```

Navigate to room 1:
```bash
ros2 action send_goal navigate_to_waypoint multimap/action/NavigateToWaypoint "{target_map: 'room1', waypoint: [-0.3,-3.3,-1.57]}"
```