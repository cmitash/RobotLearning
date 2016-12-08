# Installation 

## Installing ROS Indigo

On Ubuntu 14.04 (catkin_version):

1) Install OpenSceneGraph

    sudo apt-get install libopenscenegraph-dev

2) Follow ROS instructions for installing Indigo (http://wiki.ros.org/indigo/Installation/Ubuntu).

3) Install other ROS related packages

    sudo apt-get install ros-indigo-cmake-modules ros-indigo-fcl

4) Get PRACSYS repository

    mkdir rl_pracsys
    cd rl_pracsys
    hg clone https://USERNAME@bitbucket.org/colinmatthew/rl_pracsys src

5) Get PRACSYS Models repository

    cd ..
    hg clone https://USERNAME@bitbucket.org/colinmatthew/rl_models rl_models



6) Inside your ~/.bashrc (MUST fix paths '.....' below to match your installation):

    #ROS
    source /opt/ros/indigo/setup.bash
    export LIBRARY_PATH=$LIBRARY_PATH:/opt/ros/indigo/lib/
    export ROS_PARALLEL_JOBS=-j4

    #PRACSYS
    export PRACSYS_PATH=........ /rl_pracsys/src
    export PRACSYS_MODELS_PATH=....... /rl_models   
    export ROS_PACKAGE_PATH=$PRACSYS_PATH:$ROS_PACKAGE_PATH
    source $PRACSYS_PATH/../devel/setup.sh

    #OSG
    export OSG_FILE_PATH=$PRACSYS_MODELS_PATH
    export OSG_LIBRARY_PATH=.:/usr/local/lib:/opt/local/lib/
    export PKG_CONFIG_PATH=/usr/X11/lib/pkgconfig:$PKG_CONFIG_PATH
    export COLLADA_BOOST_FILESYSTEM_LIBRARY=/usr/local/lib

   Make sure to edit appropriately the pracsys repositories in the above lines. Then execute:

    source ~/.bashrc

7) Initialize catkin workspace

    cd rl_pracsys/src
    rm -rf CMakeLists.txt
    catkin_init_workspace 

8) Configure PRACSYS

    cd prx_build
    python build_package_list.py
    gedit packages.config

9) Edit the packages.config to load the packages that you need (0 means the package will not be compiled, 1 means the package will be compiled).
    
    # Sample packages.config file
    ('rearrangement_manipulation',0)
    ('apc',1)
    ('manipulation',1)
    ('pebble_motion',0)
    ('conformant_planning',0)
    ('coordination_manipulation',0)
    ('physics_based',0)
    ('cloud_manipulation',0)
    ('two_dim_problems',0)
    ('many_arm',0)
    ('kinematic_planners',0)
    ('homotopies',0)
    ('kinodynamic',0)
    ('decentralized_coordination',0)
    ('crowd_simulation',0)

10) Build external dependencies:

    python catkin_compile_packages.py
    cd ../prx_external 
    cmake .
    make

11) Build PRACSYS

    cd ../.. 
    catkin_make


If you want to use physics engine you need to source install Bullet with double precision: 

    cmake . -DBUILD_SHARED_LIBS=ON -DUSE_DOUBLE_PRECISION=ON


To compile, go to the pracsys directory and catkin_make

    cd $PRACSYS_PATH/.. && catkin_make

## Required Inclusion : FCL

In order to install FCL

    sudo apt-get install ros-indigo-fcl


## Required Inclusion : TRAC_IK

One of the more recent contributions is a new library for providing inverse kinematics for manipulators. The code (available at https://bitbucket.org/traclabs/trac_ik) extends KDL's IK for higher success rates.

To use this library, first install NLOpt

    sudo apt-get install libnlopt-dev

Then, you can download the trac_ik ROS package into your catkin_workspace

    git clone https://USERNAME@bitbucket.org/traclabs/trac_ik.git

Then, run these command to exclude the MoveIt specific packages

    roscd trac_ik_kinematics_plugin
    touch CATKIN_IGNORE
    roscd trac_ik_examples
    touch CATKIN_IGNORE

Next, make sure you run the catkin_compile_packages.py script again to make sure CMake finds the package.

Finally, you can rebuild your workspace, preferably running the --force-cmake option, and PRACSYS should now allow for use of this library.

# Navigating PRACSYS

# Navigating PRACSYS

PRACSYS is split into five main packages: prx_common, prx_utilities, prx_simulation, prx_planning, and prx_visualization.  Additionally, PRACSYS uses an external package (prx_external). Each package shares a similar directory structure. For example, inside prx_common the directory structure looks like this:

    ├── prx_common
    │   ├── msg
    │   │   └── ROS message files
    │   ├── prx
    │   │   └── common
    │   │       └── PRACSYS source files
    │   ├── srv
    └   └──  └── ROS service files

The msg folder has all of the message files that are maintained by the current package.  The srv folder has all of the services that are provided by the current package.  The prx folder is where all of the source files for the directory lie in.

----

# Experiments

The goal of this section is to help first-time users successfully run an experiment in PRACSYS.

## Input directory structure

prx_input is organized into two main directories: experiments and templates.  

The experiments folder is where PRACSYS experiments, in the form of roslaunch files, are run from.  It contains separate directories corresponding to different combinations of applications and task planners.  

The templates folder is where new controllers, planners, applications, and more, are specified in such a way that they can be used as templates inside of an experiment roslaunch file.  More specifically, the .yaml files which setup the rosparam server with the necessary input attributes of a class are stored.

----

## Setting up your first experiment

1. Create a main_example.launch file, which will launch your simulation, planning, and visualization launch file.

2. Create your simulation.launch file.  This file is responsible for defining all attributes of your simulation, such as which application you are running, the composition of your system tree, and many other things.  It handles launching the actual simulation ROS node.

3. Create your planning.launch file.  This file is responsible for defining all attributes of your planning, such as which task planner you are using, which motion planners you wish to run, and many other things.  It handles launching the actual planning ROS node.

4. (Optional) Create your visualization.launch file.  There is a default visualization.launch file included in PRACSYS, however if you wish to change visualization specific attributes, then this is the file to change.  It handles launching the actual visualization ROS node.

5. Running main_example.launch, assuming everything else has been written correctly, consists of simply typing "roslaunch main_example.launch", which will then start up the PRACSYS nodes you have defined in the other launch files. 

----

## Interacting with the GUI

You can find the keyboard mapping [here](https://bitbucket.org/pracsys/pracsys/wiki/User%20Interface)

----

## Recommended Practices

Queries are the fundamental way that motion planners and task planners receive input and communicate. The standard practice is PRACSYS is to construct these query objects in the parent class of the planner that resolves the query (i.e., the parent class owns the query object).

----

# Interacting with Simulation

User Interface
--------------------

PRACSYS currently uses Open Scene Graph ([OSG](http://www.openscenegraph.org/projects/osg)) to handle its visualization and user interface.

---

Keyboard Mapping
-----------------

  Key  | Function
  ------------- | -------------
  W  | Moves the camera forward (perspective) or "up" (orthographic)
  S  | Moves the camera backward (perspective) or "down" (orthographic)
  A  | Moves the camera left
  D  | Moves the camera right
  2  | Moves the camera upwards (perspective) or zooms in (orthographic)
  X  | Moves the camera upwards (perspective) or zooms in (orthographic)
  F1 - F4 | Toggles between different cameras in the scene
  +  |  Increases the camera's speed
  -  |  Decreases the camera's speed
  Spacebar | Resets the current camera to its initial position, orientation, and speed
  F  |  Toggles camera following of a selected object
  B  |  Toggles visualization of the bounding box of a selected object
  R  | Takes a screenshot of the PRACSYS window that has focus, saves in prx_output directory
  V  | Toggles video recording (continuous screenshots), saves in prx_output directory
  Enter  | Starts the simulator (if a prx_simulation node is running)
  TAB    | Steps the simulator once



---

Mouse Controls
----

Holding left-click and dragging the mouse allows control of the camera in perspective mode.  Right-clicking an object in the scene causes it to become selected, in which case the bounding box of the object is visualized.

---