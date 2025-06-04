## ❗️Symptoms

If you have **both ROS 2 Humble and Rolling installed** on your system, you may see errors like:

- `Package not found`
- `setup.bash not found`
- `3 packages failed`
- Failures, Runtime errors, Crashes

This is typically caused by **inconsistent ROS sourcing**.

---

## ✅ Solution: Remove ROS Rolling and Rebuild Cleanly

### Step 1: Uninstall ROS Rolling

```bash
sudo apt remove ~nros-rolling* -y
sudo apt autoremove -y
sudo rm -rf /opt/ros/rolling
```

### Step 2: Fix Your .bashrc
check bash: 
```bash
nano ~/.bashrc
```

remove the following line involving rolling if it exists
```bash
source /opt/ros/rolling/setup.bash
```

then reload:
```bash
source ~/.bashrc
```

### Step 3: Confirm you are only using Humble
```bash
echo $ROS_DISTRO
```
it should only return humble. 

### Step 4: Rebuild colcon workspace
```bash
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --symlink-install
```

## ✅ Debugging done! Continue with the moveit tutorial
