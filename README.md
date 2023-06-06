# relax-ros-node-reference-interpolation

Simple joint trajectory interpolation, at the moment everything is hardcoded.

The node subscribes to:
- `/xbotcore/joint_states`
- `/relax_GUI/joint_reference` incoming commands

and publishes the trajectory interpolated to:
- `/xbotcore/command`

Run it with `rosrun relax-ros-node-reference-interpolation jointTrajInterpol`
