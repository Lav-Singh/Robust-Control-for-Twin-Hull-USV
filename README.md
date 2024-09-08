# Robust-Control-for-Twin-Hull-USV
Dissertation project focusing on robust/hybrid control for a USV

To make use of the control node, make sure to first install the VRX Simulation Environment (ROS NOETIC VERSION) : https://github.com/osrf/vrx
Then either create a new working directory or drop the navigation node and model URDF file into the vrx_ws (not tested)
Follow the instructions from the VRX wiki to launch a simulation using our customised URDF
Launch the package as you would a normal ROS package

All the libraries used in this project can be obtained from PyPI. Simply install the missing packages using pip.

If adapting for another model, making use of the matlab code can be helpful to find the gain matrix and adjust Leunberger poles.
