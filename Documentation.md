# MultiMapNavigator Code Documentation

## Overview

The `MultiMapNavigator` is a ROS 2 node that enables autonomous robot navigation across multiple interconnected maps using "wormhole" transition points. It extends the standard Nav2 navigation capabilities to handle complex multi-room or multi-floor environments where a single map is insufficient.

## Class Structure

### Core Class: `MultiMapNavigator`

**Inheritance**: `rclcpp::Node`

**Primary Purpose**: Orchestrates navigation between different maps while maintaining localization and path planning capabilities.

## Key Components

### 1. Constructor and Initialization

```cpp
MultiMapNavigator::MultiMapNavigator()
```

**Functionality**:
- Initializes ROS 2 node with name "multi_map_navigator"
- Sets up publishers, clients, and action servers
- Configures map file paths relative to current working directory
- Establishes connection to Nav2 action server
- Sets initial robot pose in the starting map (room1)

**Key Features**:
- **Map Path Configuration**: Automatically discovers map files in `./maps/` directory
- **Service Connections**: Links to Nav2 navigation and map loading services
- **Initial Localization**: Places robot at coordinates (-0.3, -3.3, -1.57) in room1

### 2. Action Server Implementation

The node implements a custom action interface for waypoint navigation:

#### Goal Handling
```cpp
rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, 
                                      std::shared_ptr<const NavigateToWaypoint::Goal> goal)
```
- Accepts all incoming navigation goals
- Logs target map information

#### Goal Execution
```cpp
void execute(const std::shared_ptr<GoalHandleNavigateToWaypoint> goal_handle)
```

**Core Logic Flow**:
1. **Input Validation**: Verifies target map exists in configuration
2. **Same-Map Navigation**: Direct navigation if already in target map
3. **Multi-Map Navigation**: Complex path planning and execution across maps

### 3. Map Management

#### Map Loading
```cpp
bool load_map(const std::string &map_name)
```

**Process**:
- Validates map existence in configuration
- Calls Nav2 map server to load new map file
- Implements robust error handling with timeouts
- Ensures navigation server reconnection after map changes
- Includes stabilization delays for proper map loading

**Error Handling**:
- Service availability checks
- Timeout management (10-second limit)
- Exception catching for service failures

#### Initial Pose Setting
```cpp
void set_initial_pose(float x, float y, float yaw)
```

**Features**:
- Publishes to `/initialpose` topic for AMCL localization
- Sets appropriate covariance matrix for pose uncertainty
- Multiple publication attempts for reliability
- Quaternion conversion from Euler angles

### 4. Navigation System

#### Pose Navigation
```cpp
bool navigate_to_pose(float x, float y, float yaw)
```

**Implementation Details**:
- Creates Nav2 NavigateToPose action goals
- Implements manual polling for async operations (avoiding executor conflicts)
- 200-second timeout for navigation completion
- Comprehensive error reporting

**Robustness Features**:
- Action server connectivity verification
- Goal acceptance confirmation
- Result status validation

### 5. Path Planning Algorithm

#### Multi-Map Path Finding
```cpp
std::vector<std::string> find_path(const std::string& start, const std::string& end)
```

**Algorithm**: Depth-First Search (DFS)
- Builds adjacency graph from wormhole connections
- Finds valid path sequence through connected maps
- Returns ordered list of maps to traverse

#### DFS Implementation
```cpp
void dfs(const std::string& current, const std::string& end, ...)
```

**Features**:
- Recursive path exploration
- Visited node tracking to prevent cycles
- Backtracking capability
- First valid path return

### 6. Wormhole System

The navigation relies on a wormhole system for map transitions:

#### Wormhole Data Structure
```cpp
WormholeData wormhole_data = get_wormhole_data(from_map, to_map);
```

**Components**:
- `from_pose_x/y`: Exit coordinates in source map
- `to_pose_x/y`: Entry coordinates in destination map
- `from_yaw/to_yaw`: Robot orientations at transition points
- `valid`: Boolean indicating data availability

## Navigation Workflow

### Single-Map Navigation
1. Validate target map exists
2. Check if already in target map
3. Navigate directly to waypoint using Nav2
4. Return success/failure result

### Multi-Map Navigation
1. **Path Planning**: Calculate route through connected maps
2. **Map Traversal**: For each map transition:
   - Navigate to wormhole exit point in current map
   - Load destination map
   - Set robot pose at wormhole entry point in new map
   - Update current map tracking
   - Allow stabilization time
3. **Final Navigation**: Navigate to target waypoint in destination map

## Dependencies and Integration

### Required Services
- **Nav2 Action Server**: `/navigate_to_pose`
- **Map Server**: `/map_server/load_map`

### Published Topics
- **Initial Pose**: `/initialpose` (PoseWithCovarianceStamped)

### Action Interface
- **Custom Action**: `NavigateToWaypoint`
  - Goal: target_map (string), waypoint (float[3])
  - Result: success (bool), message (string)
  - Feedback: current_status (string)

## Configuration Requirements

### Map Files
- YAML format map files in `./maps/` directory
- Supported maps: room1, room2, room3, room4
- Proper coordinate system alignment

### Wormhole Configuration
- Bidirectional connections between maps
- Precise coordinate mapping for seamless transitions
- Consistent orientation handling

---

# Database Interface Documentation

## Overview

The `db_interface.cpp` file provides a PostgreSQL database interface for a multi-map navigation system in ROS2. It manages spatial relationships between different rooms/maps through "wormhole" connections that allow robots to transition between maps at specific coordinate points.

## Dependencies

- **pqxx**: PostgreSQL C++ library for database connectivity
- **rclcpp**: ROS2 C++ client library for logging
- **PostgreSQL**: Database server running on localhost:5432

## Database Schema

The interface connects to a PostgreSQL database named `map_db` with the following expected table structure:

```sql
CREATE TABLE multimap (
    from_room VARCHAR,      -- Source room/map identifier
    to_room VARCHAR,        -- Destination room/map identifier  
    from_pose_x FLOAT,      -- X coordinate in source map
    from_pose_y FLOAT,      -- Y coordinate in source map
    from_yaw FLOAT,         -- Orientation (yaw) in source map
    to_pose_x FLOAT,        -- X coordinate in destination map
    to_pose_y FLOAT,        -- Y coordinate in destination map
    to_yaw FLOAT           -- Orientation (yaw) in destination map
);
```

## Data Structures

### WormholeData
Represents a spatial connection between two maps:
```cpp
struct WormholeData {
    std::string from_room;   // Source room identifier
    std::string to_room;     // Destination room identifier
    float from_pose_x;       // Entry point X coordinate
    float from_pose_y;       // Entry point Y coordinate  
    float from_yaw;          // Entry orientation (radians)
    float to_pose_x;         // Exit point X coordinate
    float to_pose_y;         // Exit point Y coordinate
    float to_yaw;            // Exit orientation (radians)
    bool valid;              // Indicates if data was successfully retrieved
};
```

## Functions

### `get_wormhole_data()`

**Purpose**: Retrieves wormhole connection data between two specific rooms.

**Signature**: 
```cpp
WormholeData get_wormhole_data(const std::string &from_room, const std::string &to_room)
```

**Parameters**:
- `from_room`: Source room/map identifier
- `to_room`: Destination room/map identifier

**Returns**: `WormholeData` struct containing spatial transformation data

**Functionality**:
1. **Database Connection**: Establishes connection to PostgreSQL using hardcoded credentials
   - Database: `map_db`
   - User: `postgres`
   - Password: `password`
   - Host: `localhost:5432`

2. **Query Execution**: Uses parameterized query to prevent SQL injection:
   ```sql
   SELECT * FROM multimap WHERE from_room = $1 AND to_room = $2
   ```

3. **Data Processing**: If results found:
   - Extracts all pose data from first matching row
   - Converts database fields to appropriate data types
   - Sets `valid = true` to indicate successful retrieval

4. **Error Handling**: 
   - Catches all exceptions and logs via ROS2 logger
   - Returns invalid WormholeData on failure (valid = false)
   - Continues execution without crashing

**Usage Example**:
```cpp
WormholeData wormhole = get_wormhole_data("room1", "room2");
if (wormhole.valid) {
    // Use wormhole data for navigation
    navigate_to_pose(wormhole.from_pose_x, wormhole.from_pose_y, wormhole.from_yaw);
}
```

### `get_all_pairs()`

**Purpose**: Retrieves all available room-to-room connections for path planning.

**Signature**:
```cpp
std::vector<std::pair<std::string, std::string>> get_all_pairs()
```

**Parameters**: None

**Returns**: Vector of string pairs representing all available transitions

**Functionality**:
1. **Database Query**: Executes simple SELECT to get all connections:
   ```sql
   SELECT from_room, to_room FROM multimap
   ```

2. **Data Collection**: 
   - Iterates through all result rows
   - Creates string pairs for each connection
   - Uses `emplace_back()` for efficient insertion

3. **Error Handling**: Same pattern as `get_wormhole_data()`
   - Returns empty vector on database errors
   - Logs errors without crashing

**Usage Example**:
```cpp
auto pairs = get_all_pairs();
for (const auto& pair : pairs) {
    std::cout << "Connection: " << pair.first << " -> " << pair.second << std::endl;
}
```

## Integration with Navigation System

The database interface integrates with the multi-map navigation system (`multi_map_navigator.cpp`) as follows:

### Path Planning Integration
```cpp
// In MultiMapNavigator::find_path()
auto wormholes = get_all_pairs();  // Get all possible connections
// Build graph for pathfinding algorithm
```

### Transition Execution
```cpp
// In MultiMapNavigator::execute()
WormholeData wormhole_data = get_wormhole_data(from_map, to_map);
if (wormhole_data.valid) {
    // Navigate to exit point in current map
    navigate_to_pose(wormhole_data.from_pose_x, wormhole_data.from_pose_y, wormhole_data.from_yaw);
    
    // Load new map
    load_map(to_map);
    
    // Set robot position in new map
    set_initial_pose(wormhole_data.to_pose_x, wormhole_data.to_pose_y, wormhole_data.to_yaw);
}
```

---

This documentation covers the complete functionality of the MultiMapNavigator system and its database interface, providing insight into their architecture, algorithms, and operational characteristics for multi-map autonomous navigation.