# ROS2 Migration Plan for AIDF Core Components

## 🎯 Scope (Revised - Core Components Only)
**INCLUDED:**
- `skillgraph/` core library (except lego/ subdirectory)  
- `planner/` library 
- `exe/plan_lego.cpp` and `exe/webplan_lego.cpp` (ROS-agnostic executables)
- CMake build system

**EXCLUDED (For Later):**
- `skillgraph/src/lego/` - Defer Lego components to Phase 2
- `lego_integrate_task_planning_chaitanya.cpp` - Ignore (non-standard file)

## 📊 Current Analysis

### ROS Dependencies Found in Core Components:

**skillgraph/src/moveit_backend.cpp:** (Main target)
- `ros::init()`, `ros::NodeHandle`
- `ros::Time::now()`, `ros::Duration`  
- `ros::ServiceClient`, `ros::Publisher`

**planner/src/*.cpp:** (Secondary targets)
- `ros::Duration` (timing)
- `ros::ServiceClient` (MoveIt execution)
- `ros::ok()` (control loops)
- `ros::NodeHandle` (service clients)

## 🚀 **Focused Migration Plan**

### Phase 1: Complete ROS Abstraction Layer ✅ (In Progress)
**Goal:** Create complete ROS1/ROS2 compatibility layer for core components

**Components to Abstract:**
1. **Time & Duration** ✅ (Started in your ros_compat/time.hpp)
2. **Node Management** (New)
3. **Service Clients** (New) 
4. **Publishers** (New)

**Files to Create/Extend:**
- Complete `skillgraph/include/ros_compat/node.hpp`
- Complete `skillgraph/include/ros_compat/service_client.hpp`
- Complete `skillgraph/include/ros_compat/publisher.hpp`
- Implement backends in `skillgraph/src/ros_compat/`

### Phase 2: Update Core Components
**Goal:** Replace ROS1 calls with abstraction layer

**Files to Modify:**
- `skillgraph/src/moveit_backend.cpp` (main target) ✅
- `planner/src/tpg.cpp`, `planner/src/adg.cpp`, `planner/src/utils.cpp`

### Phase 3: Dual Build System ✅ 
**Goal:** Support both ROS1 (catkin) and ROS2 (ament)

**Tasks:**
- Create ROS2 `package.xml` ✅
- Update `CMakeLists.txt` with ROS version detection ✅
- Conditional compilation macros ✅

### Phase 4: Testing & Validation
**Goal:** Ensure both builds work correctly

## 🔧 Proposed Directory Structure
```
AIDF/
├── skillgraph/
│   ├── include/ros_compat/     # ✅ Completed
│   │   ├── time.hpp           # ✅ Exists
│   │   ├── node.hpp           # ✅ Created
│   │   ├── service_client.hpp # ✅ Created  
│   │   ├── publisher.hpp      # ✅ Created
│   │   ├── subscriber.hpp     # ✅ Created
│   │   └── launch.hpp         # ✅ Created
│   └── src/ros_compat/        # ✅ Implemented
│       ├── ros1_node.cpp      # ✅ ROS1 backends
│       ├── ros1_launch.cpp    # ✅ ROS1 launch
│       ├── ros2_node.cpp      # ✅ ROS2 backends
│       └── ros2_launch.cpp    # ✅ ROS2 launch
├── planner/                   # 🔄 Update to use ros_compat
├── package.xml               # ✅ Made ROS2 compatible
└── CMakeLists.txt            # ✅ Added ROS version detection
```

## ⏱️ **Realistic Timeline** (Core Only)
- **Phase 1:** 1-2 days (abstraction layer completion)
- **Phase 2:** 1-2 days (update core components)
- **Phase 3:** 1 day (build system)
- **Phase 4:** 1 day (testing)
- **Total:** ~1 week for core components

## 🎯 **Next Immediate Steps**
1. Complete the node abstraction interface
2. Add service client abstraction  
3. Add publisher abstraction
4. Implement ROS1 backends
5. Update `moveit_backend.cpp` to use abstractions

## 🎯 Recent Progress Update

### ✅ Completed Tasks
1. **ROS Abstraction Layer** - Complete implementation of ROS-agnostic interfaces:
   - Node management (`ros_compat/node.hpp`)
   - Service clients (`ros_compat/service_client.hpp`)
   - Publishers (`ros_compat/publisher.hpp`)
   - Subscribers (`ros_compat/subscriber.hpp`)
   - Launch system (`ros_compat/launch.hpp`)
   - Both ROS1 and ROS2 backend implementations

2. **MoveIt Backend Migration** - Updated `moveit_backend.cpp` to use abstraction layer:
   - Replaced hardcoded `roslaunch` with abstracted launcher
   - Updated ROS node initialization to use abstracted NodeFactory
   - Modified cleanup processes to work with abstracted interfaces

3. **Dual Build System** - Complete CMake and package configuration:
   - Automatic ROS version detection in `CMakeLists.txt`
   - Conditional compilation with `ROS1_BUILD`/`ROS2_BUILD` macros
   - Dual-compatible `package.xml` (format 3 with conditions)
   - Proper ament/catkin configuration

### 🔄 Next Steps
1. **Planner Integration** - Update planner components to use abstraction where needed
2. **Testing** - Validate both ROS1 and ROS2 builds
3. **Documentation** - Create build and usage instructions for both ROS versions
