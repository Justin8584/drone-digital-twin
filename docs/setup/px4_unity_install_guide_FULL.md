# PX4-Unity Digital Twin Installation & Setup Guide v2.0
*Updated: 2025-08-20*

## System Requirements

### Minimum Hardware
- CPU: Intel i5 or AMD Ryzen 5 (8th gen or newer)
- RAM: 16GB (32GB recommended)
- GPU: NVIDIA GTX 1060 or better
- Storage: 50GB free space (SSD recommended)
- OS: Windows 10 (version 2004+) or Windows 11 / Ubuntu 22.04 LTS (for WSL)

### Network Requirements
- Stable internet connection for downloads
- Firewall permissions for localhost connections
- Required ports:
  - 14550 (MAVLink/QGroundControl)
  - 8888 (MicroXRCE-DDS)
  - 10000-10005 (ROS-TCP-Connector)

## Part 1: Base System Installation

### 1.1 WSL2 Setup

#### Enable WSL2
Open PowerShell as Administrator and run:
```powershell
# Enable WSL
dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart

# Enable Virtual Machine Platform
dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart

# Restart computer
Restart-Computer
```

After restart:
```powershell
# Set WSL2 as default
wsl --set-default-version 2

# Install Ubuntu 22.04
wsl --install -d Ubuntu-22.04
```

#### Configure WSL2 Memory
Create `.wslconfig` in your Windows user directory:
```powershell
notepad $env:USERPROFILE\.wslconfig
```

Add configuration (adjust based on your RAM):
```ini
[wsl2]
memory=12GB      # For 32GB system
processors=6     # Half of your CPU cores
swap=4GB
nestedVirtualization=true

[experimental]
autoMemoryReclaim=gradual
sparseVhd=true
```

#### Update Ubuntu and Install Base Tools
```bash
# In WSL2 Ubuntu
sudo apt update && sudo apt upgrade -y

# Install essential tools
sudo apt install -y \
    curl wget git build-essential cmake \
    python3-pip python3-venv \
    software-properties-common \
    lsb-release gnupg2 x11-apps \
    net-tools lsof

# Test GUI support
xclock  # Should display a clock window
```

### 1.2 ROS2 Humble Installation

#### Add ROS2 Repository
```bash
# Add ROS2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os/release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update and install
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions
```

### 1.3 PX4 Autopilot Installation

#### Clone and Build PX4
```bash
# Create apps directory
mkdir -p ~/app
cd ~/app

# Clone PX4 repository with submodules
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# Install PX4 dependencies
bash ./Tools/setup/ubuntu.sh

# Build PX4 SITL (Software In The Loop)
make px4_sitl_default
```

#### Gazebo Installation for PX4
```bash
cd ~

# Add Gazebo repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt update
sudo apt install -y gz-garden
```

#### Test PX4 with X500 Model
```bash
cd ~/app/PX4-Autopilot
make px4_sitl gz_x500
```

### 1.4 MicroXRCE-DDS Agent Installation

#### Build from Source
```bash
# Install dependencies
sudo apt install -y git cmake build-essential

# Clone and build
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig /usr/local/lib/

# Verify installation
which MicroXRCEAgent  # Should show: /usr/local/bin/MicroXRCEAgent
```

## Part 2: ROS2 Workspace Setup

### 2.1 Create Workspace and Clone Packages
```bash
# Create ROS2 workspace
mkdir -p ~/drone_ws/src
cd ~/drone_ws/src

# Clone required packages
git clone https://github.com/PX4/px4_msgs.git -b release/1.14
git clone https://github.com/PX4/px4_ros_com.git
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git -b main-ros2
```

### 2.2 Create Custom Unity Bridge Package
```bash
# Create package structure
mkdir -p ~/drone_ws/src/drone_unity_bridge/launch
mkdir -p ~/drone_ws/src/drone_unity_bridge/scripts

# Create package.xml
cat << 'EOF' > ~/drone_ws/src/drone_unity_bridge/package.xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>drone_unity_bridge</name>
  <version>0.0.1</version>
  <description>Unity bridge for drone simulation</description>
  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>MIT</license>
  
  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>ros_tcp_endpoint</exec_depend>
  <exec_depend>px4_msgs</exec_depend>
  
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF

# Create setup.py
cat << 'EOF' > ~/drone_ws/src/drone_unity_bridge/setup.py
from setuptools import setup
import os
from glob import glob

package_name = 'drone_unity_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Unity bridge for drone simulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
    scripts=['scripts/qos_relay.py'],
)
EOF

# Create resource folder
mkdir -p ~/drone_ws/src/drone_unity_bridge/resource
touch ~/drone_ws/src/drone_unity_bridge/resource/drone_unity_bridge
```

### 2.3 Create Launch File
```bash
cat << 'EOF' > ~/drone_ws/src/drone_unity_bridge/launch/unity_endpoint.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'tcp_ip',
            default_value='0.0.0.0',
            description='IP address for TCP endpoint'
        ),
        DeclareLaunchArgument(
            'tcp_port',
            default_value='10000',
            description='Port for TCP endpoint'
        ),
        LogInfo(msg=['Starting Unity TCP Endpoint on port ', LaunchConfiguration('tcp_port')]),
        Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            name='unity_endpoint',
            output='screen',
            parameters=[{
                'ROS_IP': LaunchConfiguration('tcp_ip'),
                'ROS_TCP_PORT': LaunchConfiguration('tcp_port'),
            }]
        )
    ])
EOF
```

### 2.4 Create QoS Relay Script (Critical Component)
```bash
cat << 'EOF' > ~/drone_ws/src/drone_unity_bridge/scripts/qos_relay.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleLocalPosition

class QoSRelay(Node):
    def __init__(self):
        super().__init__('qos_relay_node')
        
        # QoS for subscribing to PX4 (best_effort)
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # QoS for publishing to Unity (reliable)
        unity_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to PX4 topic with best_effort
        self.subscription = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.relay_callback,
            px4_qos
        )
        
        # Publish for Unity with reliable
        self.publisher = self.create_publisher(
            VehicleLocalPosition,
            '/unity/vehicle_local_position',
            unity_qos
        )
        
        self.get_logger().info('QoS Relay started: PX4 -> Unity')
        self.msg_count = 0
        
    def relay_callback(self, msg):
        self.publisher.publish(msg)
        self.msg_count += 1
        if self.msg_count % 50 == 0:
            self.get_logger().info(f'Relayed {self.msg_count} messages')

def main(args=None):
    rclpy.init(args=args)
    node = QoSRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

# Make script executable
chmod +x ~/drone_ws/src/drone_unity_bridge/scripts/qos_relay.py
```

### 2.5 Build Workspace
```bash
cd ~/drone_ws

# Build packages in order
colcon build --packages-select px4_msgs
source install/setup.bash

colcon build --packages-select px4_ros_com
source install/setup.bash

colcon build --packages-select ros_tcp_endpoint
source install/setup.bash

colcon build --packages-select drone_unity_bridge
source install/setup.bash
```

## Part 3: Unity Setup

### 3.1 Install Unity Hub (Windows)
```powershell
# Download Unity Hub
curl -o UnityHubSetup.exe https://public-cdn.cloud.unity3d.com/hub/prod/UnityHubSetup.exe

# Install Unity Hub
.\UnityHubSetup.exe
```

### 3.2 Install Unity 2022.3 LTS
1. Open Unity Hub
2. Go to "Installs" → "Install Editor"
3. Select Unity 2022.3.XX LTS
4. Add modules:
   - Windows Build Support (IL2CPP)
   - Microsoft Visual Studio Community 2022

### 3.3 Create Project and Install ROS-TCP-Connector
1. Create new 3D project: "DroneSimulationViz"
2. Open Package Manager (Window → Package Manager)
3. Click "+" → "Add package from git URL"
4. Add: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git`

### 3.4 Generate PX4 Messages in Unity
1. Copy message definitions from WSL to Windows:
   ```bash
   cp -r ~/drone_ws/src/px4_msgs/msg /mnt/c/Users/[YourUsername]/Desktop/px4_msgs
   ```
2. In Unity: **Robotics → Generate ROS Messages**
3. Browse to the copied px4_msgs folder
4. Click **Build Messages**

### 3.5 Configure ROS Settings
1. Go to **Robotics → ROS Settings**
2. Set:
   - ROS IP Address: `127.0.0.1`
   - ROS Port: `10000`
   - Protocol: `ROS2`

### 3.6 Create Drone Subscriber Script
Create `DronePX4Subscriber.cs` in Assets/Scripts:
```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Px4;

public class DronePX4Subscriber : MonoBehaviour
{
    private ROSConnection ros;
    private bool isReceivingData = false;
    private float lastMessageTime = 0f;
    private int messageCount = 0;
    
    private Vector3 targetPosition;
    private Vector3 currentVelocity;
    public float smoothTime = 0.1f;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        // Subscribe to RELAYED topic (not direct PX4 topic)
        ros.Subscribe<VehicleLocalPositionMsg>("/unity/vehicle_local_position", PositionCallback);
        Debug.Log("Subscribed to: /unity/vehicle_local_position");
        targetPosition = transform.position;
    }
    
    void PositionCallback(VehicleLocalPositionMsg msg)
    {
        messageCount++;
        lastMessageTime = Time.time;
        isReceivingData = true;
        
        if (messageCount % 10 == 0)
        {
            Debug.Log($"[MSG #{messageCount}] Position - X: {msg.x:F2}, Y: {msg.y:F2}, Z: {msg.z:F2}");
        }
        
        // Convert NED to Unity coordinates
        targetPosition = new Vector3(msg.y, -msg.z, msg.x);
    }
    
    void Update()
    {
        transform.position = Vector3.SmoothDamp(
            transform.position,
            targetPosition,
            ref currentVelocity,
            smoothTime
        );
        
        if (Time.time - lastMessageTime > 2f && isReceivingData)
        {
            Debug.LogWarning("No messages received for 2 seconds!");
            isReceivingData = false;
        }
    }
    
    void OnGUI()
    {
        GUIStyle style = new GUIStyle();
        style.fontSize = 16;
        style.normal.textColor = isReceivingData ? Color.green : Color.red;
        string status = isReceivingData ? 
            $"RECEIVING DATA (#{messageCount})" : "NO DATA";
        GUI.Label(new Rect(10, 10, 300, 30), $"ROS Status: {status}", style);
        GUI.Label(new Rect(10, 40, 300, 30), $"Position: {transform.position}", style);
    }
}
```

## Part 4: Environment Configuration

### 4.1 Configure .bashrc for Auto-sourcing
```bash
# Add to ~/.bashrc
echo "# ROS2 Humble setup" >> ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "" >> ~/.bashrc
echo "# Drone workspace" >> ~/.bashrc
echo "source ~/drone_ws/install/setup.bash 2>/dev/null" >> ~/.bashrc
echo "" >> ~/.bashrc
echo "# DDS Configuration for PX4" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc
echo "" >> ~/.bashrc
echo "# Helpful aliases" >> ~/.bashrc
echo "alias px4_start='cd ~/app/PX4-Autopilot && make px4_sitl gz_x500'" >> ~/.bashrc
echo "alias drone_bridge='ros2 launch drone_unity_bridge unity_endpoint.launch.py'" >> ~/.bashrc
echo "alias check_topics='ros2 topic list | grep fmu'" >> ~/.bashrc
echo "alias echo_position='ros2 topic echo /fmu/out/vehicle_local_position_v1 --qos-reliability best_effort'" >> ~/.bashrc

# Reload bashrc
source ~/.bashrc
```

## Part 5: Running the Complete System

### 5.1 Startup Sequence (CRITICAL ORDER)

#### Terminal 1: PX4 SITL with Gazebo
```bash
cd ~/app/PX4-Autopilot
make px4_sitl gz_x500

# Wait for "Ready for takeoff!"
# Verify DDS client:
uxrce_dds_client status
```

#### Terminal 2: MicroXRCE-DDS Agent
```bash
MicroXRCEAgent udp4 -p 8888

# Look for "session established"
```

#### Terminal 3: QoS Relay (CRITICAL)
```bash
python3 ~/drone_ws/src/drone_unity_bridge/scripts/qos_relay.py

# Should show "Relayed X messages" continuously
```

#### Terminal 4: Unity TCP Bridge
```bash
ros2 launch drone_unity_bridge unity_endpoint.launch.py

# Shows "Starting server on 0.0.0.0:10000"
```

#### Unity Application
1. Open Unity project
2. Attach DronePX4Subscriber to Drone GameObject
3. Press Play
4. Should show green "RECEIVING DATA"

#### Terminal 1: Execute Commands
```bash
commander arm -f      # Arm drone
commander takeoff     # Rise to 2.5m
commander land        # Return to ground
```

## Part 6: Troubleshooting

### Common Issues and Solutions

#### Port 8888 Already in Use
```bash
# Check what's using port
sudo lsof -i :8888
# Kill process
sudo fuser -k 8888/udp
```

#### Topics Not Appearing
- PX4 publishes to `/fmu/out/*` NOT `/px4/fmu/out/*`
- Topics have `_v1` suffix (e.g., `vehicle_local_position_v1`)

#### QoS Incompatibility
- PX4 uses `best_effort` reliability
- Unity expects `reliable`
- QoS relay solves this mismatch

#### Unity Shows "NO DATA"
1. Check QoS relay is running and showing messages
2. Verify Unity subscribes to `/unity/vehicle_local_position`
3. Check all terminals are running in correct order

#### Build Errors in drone_ws
```bash
# Clean and rebuild
cd ~/drone_ws
rm -rf build/ install/ log/
colcon build
source install/setup.bash
```

### Verification Commands
```bash
# Check topics exist
ros2 topic list | grep -E "fmu|unity"

# Check data flow
ros2 topic hz /unity/vehicle_local_position

# Monitor position
ros2 topic echo /unity/vehicle_local_position --once
```

## Key Architecture Notes

1. **Topic Namespace**: PX4 publishes to `/fmu/out/*` not `/px4/fmu/out/*`
2. **Message Suffix**: Topics include `_v1` suffix
3. **QoS Mismatch**: Relay required to convert best_effort → reliable
4. **Startup Order**: Must start components in exact sequence
5. **Unity Topic**: Subscribe to `/unity/vehicle_local_position` (relayed)

## Success Indicators

| Component | Success Indicator |
|-----------|-------------------|
| PX4 | `Payload tx: ~41000 B/s` |
| MicroXRCE | `session established` |
| QoS Relay | `Relayed X messages` |
| Unity Bridge | `Connection from 127.0.0.1` |
| Unity | Green "RECEIVING DATA" |

## Next Steps

- Add attitude/rotation visualization
- Implement sensor overlays
- Create mission planning interface
- Add multi-drone support
- Integrate wildlife detection simulation

---

*Version 2.0 includes QoS relay solution and complete troubleshooting guide based on successful implementation.*