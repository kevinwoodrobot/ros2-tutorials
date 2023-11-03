# Problem - ros2: command not found
Common error when running any `ros2` commands. 

# Solution - Sourcing Setup.bash
**1. Command way:** Run `echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc` to automatically source. This lets your `ros2` command work. 

**2. Preffered way:** Go to your `.bashrc` file and manually add `source /opt/ros/humble/setup.bash` in VS Code. 

# Next Video:<br>Running Executables