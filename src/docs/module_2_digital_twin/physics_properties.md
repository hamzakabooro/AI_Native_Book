# Physics Property Configuration in Gazebo

## Overview
This guide covers how to configure physics properties in Gazebo simulation to ensure realistic behavior of your digital twin. Properly configured physics properties are essential for accurate simulation that reflects real-world robot behavior.

## Physics Engines in Gazebo

Gazebo uses different physics engines to simulate rigid body dynamics:
- **ODE (Open Dynamics Engine)**: Default engine, good for most applications
- **Bullet**: Alternative engine with different performance characteristics
- **SimBody**: Less commonly used but available

## Configuring World Physics Properties

### Setting Global Parameters
Physics parameters for the entire world are configured in the SDF world file:

```xml
<sdf version="1.7">
  <world name="physics_world">
    <!-- Physics engine configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>  <!-- Time step for simulation -->
      <real_time_factor>1.0</real_time_factor>  <!-- Simulation speed relative to real time -->
      <real_time_update_rate>1000.0</real_time_update_rate>  <!-- Updates per second -->
      <gravity>0 0 -9.8</gravity>  <!-- Gravity vector (x, y, z) -->
    </physics>
    
    <!-- Your models here -->
  </world>
</sdf>
```

### Time Step and Real-Time Factor
- **max_step_size**: The time increment for each simulation step (typically 0.001s)
- **real_time_factor**: How fast simulation runs compared to real time (1.0 = real-time)
- **real_time_update_rate**: How often the simulation updates (Hz)

Lower time steps increase accuracy but decrease performance.

## Configuring Link Physics Properties

### Inertial Properties
For each link in your URDF/SDF, you need to properly define inertial properties:

```xml
<link name="link_name">
  <!-- Visual and collision properties -->
  <visual>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </collision>
  
  <!-- Mass and inertial properties -->
  <inertial>
    <mass value="0.5"/>  <!-- Mass in kilograms -->
    <inertia 
      ixx="0.001" ixy="0.0" ixz="0.0"
      iyy="0.001" iyz="0.0"
      izz="0.001"/>  <!-- Moments of inertia -->
  </inertial>
</link>
```

### Understanding Moments of Inertia
The moments of inertia define how mass is distributed in the object:
- `ixx`, `iyy`, `izz`: Resistance to rotation around x, y, and z axes respectively
- `ixy`, `ixz`, `iyz`: Products of inertia (usually 0 for symmetric objects)

For common shapes:
- Box: `I = 1/12 * m * (h² + d²)` for axis parallel to length
- Cylinder: `I_around_center_axis = 1/2 * m * r²`, `I_around_perpendicular = 1/12 * m * (3*r² + h²)`
- Sphere: `I = 2/5 * m * r²`

## Surface Properties and Friction

### Contact Properties
Gazebo allows detailed configuration of surface properties for realistic interaction:

```xml
<gazebo reference="link_name">
  <collision>
    <surface>
      <friction>
        <!-- ODE friction model -->
        <ode>
          <mu>1.0</mu>      <!-- Primary friction coefficient -->
          <mu2>1.0</mu2>    <!-- Secondary friction coefficient -->
          <fdir1>0 0 1</fdir1>  <!-- Primary friction direction -->
          <slip1>0.0</slip1>    <!-- Primary slip coefficient -->
          <slip2>0.0</slip2>    <!-- Secondary slip coefficient -->
        </ode>
        
        <!-- Bullet friction model -->
        <bullet>
          <friction>1.0</friction>
          <friction2>1.0</friction2>
          <fdir1>0 0 1</fdir1>
          <rolling_friction>0.0</rolling_friction>
        </bullet>
      </friction>
      
      <!-- Contact parameters -->
      <contact>
        <ode>
          <soft_cfm>0.0</soft_cfm>      <!-- Constraint Force Mixing -->
          <soft_erp>0.2</soft_erp>      <!-- Error Reduction Parameter -->
          <kp>1e+13</kp>                <!-- Spring stiffness -->
          <kd>1.0</kd>                  <!-- Damping coefficient -->
          <max_vel>0.01</max_vel>       <!-- Maximum contact correction velocity -->
          <min_depth>0.0</min_depth>    <!-- Minimum contact depth -->
        </ode>
      </contact>
      
      <!-- Bounce properties -->
      <bounce>
        <restitution_coefficient>0.1</restitution_coefficient>  <!-- Bounciness (0-1) -->
        <threshold>100000.0</threshold>  <!-- Minimum velocity for bounce -->
      </bounce>
    </surface>
  </collision>
</gazebo>
```

## Practical Example: Configuring a Wheeled Robot

Let's see how to properly configure physics for our wheeled robot:

```xml
<?xml version="1.0"?>
<robot name="physics_robot">

  <!-- Base link with realistic properties -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.15"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>  <!-- Heavier base for stability -->
      <inertia ixx="0.0854" ixy="0.0" ixz="0.0" 
               iyy="0.1042" iyz="0.0" 
               izz="0.0208"/>
    </inertial>
  </link>

  <!-- Wheel with proper physics -->
  <link name="wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.0013" ixy="0.0" ixz="0.0"
               iyy="0.0013" iyz="0.0"
               izz="0.0020"/>  <!-- Higher inertia around rotation axis -->
    </inertial>
  </link>

  <!-- Joint between base and wheel -->
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel"/>
    <origin xyz="0.2 0.175 -0.025" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Gazebo-specific physics properties -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="wheel">
    <material>Gazebo/Black</material>
    <!-- Wheel friction for realistic movement -->
    <mu1>1.0</mu1>  <!-- Forward/backward friction -->
    <mu2>1.0</mu2>  <!-- Side friction (lower for wheels) -->
    <!-- Contact properties for stable simulation -->
    <kp>1000000.0</kp>  <!-- Spring stiffness -->
    <kd>100.0</kd>      <!-- Damping coefficient -->
    <!-- ODE-specific settings -->
    <fdir1>0 0 1</fdir1>  <!-- Friction direction -->
    <max_vel>1.0</max_vel>
    <min_depth>0.001</min_depth>
  </gazebo>

</robot>
```

## Physics Debugging and Validation

### Common Physics Issues and Solutions

1. **Robot falls through ground or objects**
   - Check that collision meshes are properly defined
   - Verify that inertial properties are balanced
   - Ensure that all required geometric properties are defined

2. **Robot explodes or moves erratically**
   - Check that mass values are positive and reasonable
   - Verify inertial values follow the parallel axis theorem
   - Ensure moments of inertia are positive

3. **Simulation runs slowly**
   - Increase the time step (max_step_size)
   - Reduce model complexity
   - Simplify collision meshes

4. **Joints behave strangely or vibrate**
   - Verify joint limits and positions
   - Check contact properties (kp, kd settings)
   - Ensure reasonable mass ratios between connected links

### Physics Validation Checklist

- [ ] All links have positive mass values
- [ ] Moments of inertia satisfy the triangle inequality (Ix + Iy ≥ Iz, etc.)
- [ ] Center of mass is within the physical bounds of the link
- [ ] Contact properties (kp, kd) are reasonable for the material
- [ ] Friction coefficients match real-world materials
- [ ] Collision meshes encompass visual meshes

## Performance Optimization

### Balancing Accuracy and Performance
- Use simpler collision meshes than visual meshes
- Choose appropriate time step values (0.001s is typically a good starting point)
- Adjust real-time factor based on computational resources
- Limit the number of contacts in complex scenarios

### Contact Reduction
- Simplify models to reduce the number of contact points
- Use appropriate contact resolution parameters
- Consider using larger time steps for less critical simulations

## Advanced Physics Concepts

### Custom Physics Plugins
For advanced physics behavior, you can create custom Gazebo plugins:

```cpp
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
  class CustomPhysicsPlugin : public WorldPlugin
  {
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
      // Custom physics implementation
    }
  };
  GZ_REGISTER_WORLD_PLUGIN(CustomPhysicsPlugin)
}
```

## Best Practices

1. **Realistic Mass Distribution**: Ensure your robot's mass properties match its real counterpart
2. **Conservative Friction**: Start with realistic friction coefficients (0.5-1.0 for most materials)
3. **Validation**: Test physics behavior with various scenarios before using in complex algorithms
4. **Documentation**: Keep notes on how you calculated physics properties for future reference
5. **Iterative Approach**: Adjust physics properties incrementally and test frequently

## Summary

Properly configured physics properties are crucial for realistic Gazebo simulations. By understanding how to set inertial properties, friction coefficients, and contact parameters, you can create digital twins that accurately reflect real-world behavior.

## Resources
- [Gazebo Physics Documentation](http://gazebosim.org/tutorials?tut=physics)
- [URDF Inertial Calculator](https://www.gazebo-sim.org/tutorials?tut=inertial_calculator)
- [ODE Physics Parameters](https://www.ode-wiki.org/wiki/index.php?title=Manual)