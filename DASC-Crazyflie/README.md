## Acknowledgments

This project is based on [Crazyswarm](https://github.com/USC-ACTLab/crazyswarm) developed by USC ACT Lab. The original code is licensed under the MIT License, which is included in this repository.

# Setup Guide
1. Download Docker from [here](https://docs.docker.com/engine/install/ubuntu/)
2. Get into the directory where `Dockerfile` is located
3. On the command prompt, type the following commands
```
xhost +
docker compose build
docker compose up -d
docker exec -it {name of the folder}-ros-1 bash
cd ~/crazyswarm
./build.sh
```
# How to Start
After it is built, when you want to run the docker container on the command prompt in future, you can simply type the following commands.
```
xhost +
docker compose up -d
docker exec -it {name of the folder}-ros-1 bash
```

For more information, you can look at the instructions in [here](https://crazyswarm.readthedocs.io/en/latest/installation.html)
# How to Use
1. All the scripts are located in the directory `scripts` folder in `~/crazyswarm/ros_ws/src/crazyswarm/scripts`
You can access this folder by typing 
```
cd ~/crazyswarm/ros_ws/src/crazyswarm/scripts
```
or
```
cd $SHORT
```

2. In order to run the crazyswarm node, we need to type
```
roslaunch crazyswarm hover_swarm.launch
```
If you are running inside Docker, SSH, or any session without a working X display, launch headless to avoid the Qt `xcb` crash:
```
roslaunch crazyswarm hover_swarm.launch use_rviz:=false
```
If you do want RViz and have a working display, use:
```
roslaunch crazyswarm hover_swarm.launch use_rviz:=true
```
3. Once the node is running, open another command propt and get into the docker file again, go to the `scripts` folder, and run the python script
4. In case you would like to just run the simulation, type `python3 'python_script_name'.py --sim`

# Flashing Firmwares 
## Crazyflies (Cannot flash multiple crazyflies at the same time?)
1. When turing on the crazyflie, press its power button for 3 seconds to put it into the bootloader mode
2. Turn the CFclient, and go to Connect -> Bootloader -> Cold boot (recovery)
3. Initiate bootloader cold boot -> Program
4. Wait until everything is finished

## Crazyradio PA/2.1
1. Go to [here](https://github.com/bitcraze/crazyradio-firmware)
2. Clone the repo and follow the instructions (ours is Crazyradio PA)
3. Note we have 4 crazyradio PAs flahsed and working
4. Please install the necessary drivers and allow the radios via [here](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/install/) 

# Flying Drones
1. First, you want to set up the initial positions of the crazyflies. We can do this by modifying `allCrazyflies.yaml` file in `~/crazyswarm/ros_ws/src/crazyswarm/launch/` 
2. In one comamnd prompt, go to the `scripts` folder and run `chooser.py` script to select and deselect the crazyflies you would like to include in deployment. Selecting and Deselecting modifies `crazyflies.yaml` file in the same folder
3. Launch the crazyswarm node 
4. Now you are ready to run your script
5. To test hovering and landing, you can run `first_test.py` in the `scripts` folder.

# Emegency Shutdown
1. We have `emergency_break.py` that constantly publishes the status=True of `Crazyswarm` class
2. If you interrupt `emergency_break.py`, it will start publishing status=False, which can be used to terminate the ongoing script
3. Run the script while running your ros node, and interrupt the script if you want to land the drones straight down

# Initial Position Landing
1. `crazyswarm_py.py` has `return_initial_controller` function, which brings back the drones back to their initial positions
2. Currently, it is set that if the ros node crashes, or the script terminates, the `return_initial_controller` function is automatically called 


# Common Debugging
In case you get an error with an exit code -6 after launching the crazyswarm node, make sure 
1. You have typed ```xhost +``` in a command prompt
2. If the error mentions Qt or `xcb`, run `roslaunch crazyswarm hover_swarm.launch use_rviz:=false`
3. You are connected to the right WiFi
4. Vicon cameras are on and can sense the crazyflies (one way to check is run `roslaunch crazyswarm mocap_helper.launch`)
5. You are connected to the right vicon system (You can check this by looking at the entry for `motion_capture_host_name` in `hover_swarm.yaml` file)
6. Your chooser GUI is not on
7. Your computer can recognize the Crazyradio PA, and the radio is also correctly flashed (the flashed ones have `ok` stickers)
8. You have connected enough Crazyradios, i.e. if you have 3 different channels, you need 3 radios
9. If one crazyflie flips over while another crazyflie goes wild, check their numbers and locations. They might have been flipped

If the crazyflies flip over as soon as they take off, check 
1. Crazyradio PA is correctly flashed (the flashed ones have `ok` stickers)
2. The propellers are placed correctly (check [here](https://www.bitcraze.io/documentation/tutorials/getting-started-with-crazyflie-2-x/) for more information)
3. The crazyflies are using the RIGHT propellers (check [here](https://store.bitcraze.io/products/propeller-pack))
4. Crazyflies are using the PID controller when doing velocity commands (You can check this by looking at the entry for `controller` in `hover_swarm.launch` file)
5. Battery wires are not blocking the propellers (yeah, seriously. I learned it the hard way)

If the crazyflies suddenly flip over while flying
1. Make sure you are not using goTo and cmdVelocityWorld together
