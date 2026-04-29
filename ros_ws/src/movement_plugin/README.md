# 3D Movement Gazebo Plugin

The `3d_movement` plugin is a Gazebo simulation plugin designed to give you direct control over a model's kinematics. It allows you to move your model by publishing ROS 2 `Twist` messages to a `/cmd_vel` topic, applying linear and angular velocities relative to the body's North-East-Down (NED) reference frame.

This architecture provides immense flexibility: you can implement any custom dynamical model and easily integrate it into Gazebo simply by publishing the resulting velocity commands. Then, you can then measure the model's actual simulation states using the `/odom` topic or any other state estimation system. To apply this plugin inside your robot URDF model, include the following block:
``` xml
<gazebo>
	<plugin name="movement_3d_plugin" filename="libmovement_3d_plugin.so">
		<cmdVelTopic>cmd_vel</cmdVelTopic>
		<statesFrame>robot_name</statesFrame>
		<robotBaseFrame>base_link</robotBaseFrame>
		<update_rate>10</update_rate>
	</plugin>
</gazebo>
```

**Configuration Parameters:**

- **`<cmdVelTopic>`**: The ROS 2 topic used to receive `Twist` messages containing your velocity commands (e.g., `cmd_vel`).
- **`<statesFrame>`**: The specific name of your model within the Gazebo simulation.
- **`<robotBaseFrame>`**: The base reference frame of your model, exactly as it is represented in your URDF (e.g., `base_link`).
- **`<update_rate>`**: The frequency (in Hz) at which the plugin updates the model's movement.
