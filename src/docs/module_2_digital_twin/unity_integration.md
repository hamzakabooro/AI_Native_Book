# Unity Integration for Visualization

## Overview
This module covers integrating robotics simulation with Unity for enhanced visualization and user interaction. Unity provides high-quality 3D visualization capabilities that complement Gazebo and Isaac Sim simulations.

## Learning Objectives
- Understand Unity's role in robotics visualization
- Set up Unity for robotics simulation
- Connect Unity to ROS 2 for real-time visualization
- Create interactive visualization interfaces

## Unity in Robotics Context

### Advantages of Unity for Robotics
Unity offers several advantages for robotics visualization:
- **High-quality rendering**: Physically-based rendering with realistic lighting
- **Interactive interfaces**: User-friendly interfaces for teleoperation and monitoring
- **Cross-platform deployment**: Deploy to various platforms including VR/AR
- **Asset ecosystem**: Large library of 3D models and environments
- **Animation system**: For visualizing robot movements and behaviors

### Comparison with Other Tools
| Feature | Unity | Gazebo | Isaac Sim |
|---------|-------|--------|-----------|
| Visual Quality | Very High | Medium | Very High |
| Physics Simulation | Basic | Advanced | Advanced |
| ROS Integration | Moderate | Native | Native |
| User Interaction | Excellent | Basic | Good |

## Setting up Unity for Robotics

### Prerequisites
- Unity Hub and Unity 2021.3 LTS or later
- Unity Visual Scripting package (optional)
- ROS# package for ROS integration
- Robot Operating System (ROS 2 Humble Hawksbill)

### Installation Process
1. **Install Unity Hub** from the official website
2. **Install Unity Editor** with the Universal Render Pipeline (URP) or High Definition Render Pipeline (HDRP)
3. **Install ROS# package** through Unity's Package Manager or import manually

### ROS# Integration
ROS# (ROS Sharp) provides the interface between Unity and ROS 2:

1. **Import ROS# package** into your Unity project
2. **Configure ROS Settings** in Unity:
   - Set ROS_MASTER_URI (default: http://localhost:11311)
   - Set ROS_IP (your local IP address, if using remote ROS master)

## Creating a Basic Unity Visualization

### Project Setup
1. Create a new Unity 3D project
2. Import the ROS# package
3. Set up the basic scene structure:
   - Main Camera for viewing
   - Robot model imported from URDF (if available)
   - Lighting setup

### Basic Robot Visualization
```csharp
using UnityEngine;
using RosSharp.RosBridgeClient;

public class RobotVisualizer : MonoBehaviour
{
    public Transform robotBase;
    public Transform[] joints; // Array of joint transforms
    public string[] jointNames; // Names of joints in ROS
    
    private RosSocket rosSocket;
    
    void Start()
    {
        // Initialize ROS connection
        WebSocketSimpleClient webSocket = new WebSocketSimpleClient("ws://localhost:9090");
        rosSocket = new RosSocket(webSocket);
        
        // Subscribe to joint state topic
        rosSocket.Subscribe<sensor_msgs.JointState>(
            "/joint_states", 
            JointStateHandler, 
            10
        );
    }
    
    void JointStateHandler(sensor_msgs.JointState jointState)
    {
        for (int i = 0; i < jointState.name.Count; i++)
        {
            // Find the corresponding joint in our Unity model
            for (int j = 0; j < jointNames.Length; j++)
            {
                if (jointState.name[i] == jointNames[j])
                {
                    // Update the joint rotation based on ROS data
                    joints[j].localRotation = Quaternion.Euler(
                        0, 
                        0, 
                        Mathf.Rad2Deg * (float)jointState.position[i]
                    );
                    break;
                }
            }
        }
    }
    
    void OnDestroy()
    {
        rosSocket.Close();
    }
}
```

## Advanced Unity Integration Techniques

### 1. Real-time Sensor Visualization
Visualize sensor data from ROS topics:
- Point clouds from LiDAR sensors
- Depth information from cameras
- Force/torque sensor feedback

### 2. Custom Controllers
Create Unity-based controllers for:
- Robot teleoperation
- Simulation control
- Experiment management

### 3. AR/VR Integration
Deploy Unity visualizations to AR/VR platforms:
- Oculus/Meta headsets
- Microsoft HoloLens
- Mobile AR platforms

## Unity-Specific Implementation Examples

### Unity-ROS Bridge Setup
For Unity to communicate with ROS 2:

1. **Set up rosbridge_server** on the ROS side:
   ```bash
   # Terminal 1: Start ROS core
   ros2 run rosbridge_server rosbridge_websocket --port 9090
   ```

2. **Configure Unity to connect** to rosbridge:
   - Create a WebSocket connection in Unity
   - Register publishers and subscribers for needed topics

### Visualization Pipeline
The visualization pipeline typically involves:
1. **Data Acquisition**: Subscribe to sensor topics in ROS
2. **Data Processing**: Transform ROS data to Unity coordinate system
3. **Visual Rendering**: Render data in Unity's 3D environment
4. **User Interaction**: Handle user input and send commands back to ROS

## Unity Visualization Examples

### 1. Point Cloud Visualization
```csharp
using UnityEngine;
using System.Collections.Generic;

public class PointCloudVisualizer : MonoBehaviour
{
    public GameObject pointPrefab; // Small sphere prefab
    private List<GameObject> currentPoints = new List<GameObject>();
    
    public void UpdatePointCloud(float[,] points)
    {
        // Clear previous points
        foreach (GameObject point in currentPoints)
        {
            DestroyImmediate(point);
        }
        currentPoints.Clear();
        
        // Create new points
        for (int i = 0; i < points.GetLength(0); i++)
        {
            GameObject point = Instantiate(pointPrefab);
            point.transform.position = new Vector3(
                points[i, 0], // x
                points[i, 1], // y
                points[i, 2]  // z
            );
            currentPoints.Add(point);
        }
    }
}
```

### 2. Interactive Robot Control
```csharp
using UnityEngine;
using UnityEngine.EventSystems;

public class RobotController : MonoBehaviour, IPointerDownHandler
{
    public Camera mainCamera;
    public string rosTopic = "/cmd_vel";
    
    public void OnPointerDown(PointerEventData eventData)
    {
        // Raycast to determine where user clicked
        Ray ray = mainCamera.ScreenPointToRay(eventData.position);
        RaycastHit hit;
        
        if (Physics.Raycast(ray, out hit))
        {
            // Send command to robot based on click position
            SendRobotCommand(hit.point);
        }
    }
    
    void SendRobotCommand(Vector3 targetPosition)
    {
        // In a real implementation, this would publish to ROS
        Debug.Log($"Sending robot to position: {targetPosition}");
    }
}
```

## Best Practices for Unity Integration

### Performance Optimization
1. **LOD Systems**: Use Level of Detail to optimize rendering
2. **Object Pooling**: Reuse objects instead of instantiating/destroying
3. **Occlusion Culling**: Don't render objects not visible to camera
4. **Texture Compression**: Use appropriate texture formats for performance

### Data Synchronization
1. **Timestamp Handling**: Synchronize Unity and ROS timestamps
2. **Interpolation**: Smooth transitions between discrete ROS updates
3. **Buffer Management**: Handle variable message arrival times

### User Experience
1. **Intuitive Controls**: Design user-friendly interfaces
2. **Visual Feedback**: Provide clear feedback for robot status
3. **Error Handling**: Gracefully handle connection issues

## Safety Considerations in Unity

### Visual Safety Indicators
- Highlight potential collision zones
- Show robot operational boundaries
- Display safety zone areas

### Simulation Boundaries
- Clearly define and visualize simulation boundaries
- Prevent user actions that could cause unsafe robot behavior
- Implement safety stops in the Unity interface

## Integration with Other Modules

### Link to Gazebo Simulation
- Use Unity as a visualization overlay for Gazebo
- Synchronize robot states between both environments
- Share environments and objects between tools

### Link to Isaac Sim
- Import Isaac Sim environments into Unity
- Visualize Isaac Sim sensor data in Unity
- Use Unity for human-robot interaction in Isaac Sim scenarios

## Troubleshooting Common Issues

### Connection Problems
- Verify ROS bridge is running and accessible
- Check IP addresses and firewall settings
- Confirm WebSocket connections are properly established

### Performance Issues
- Reduce polygon count in 3D models
- Limit the number of simultaneous visualized elements
- Optimize texture sizes and usage

### Synchronization Problems
- Verify coordinate system alignment
- Check timing and frequency of data updates
- Ensure message formats are correctly handled

## Resources and Further Learning

- [Unity Robotics Hub](https://unity.com/solutions/industrial/robotics)
- [ROS# GitHub Repository](https://github.com/Unity-Technologies/ROS-TCP-Connector)
- [Unity Robotics Package Documentation](https://github.com/Unity-Technologies/ROS-TCP-Endpoint)
- [Unity Manual](https://docs.unity3d.com/Manual/index.html)

## Summary
Unity integration provides powerful visualization capabilities for robotics applications. By combining Unity's rendering and interaction capabilities with ROS's robotics infrastructure, we can create intuitive, high-quality interfaces for robot monitoring and teleoperation.

## Exercise
1. Set up a basic Unity project with ROS# integration
2. Create a simple robot visualization
3. Connect to a running ROS 2 simulation
4. Implement basic robot control through Unity interface