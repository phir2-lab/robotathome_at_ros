# robotathome_at_ros package

This package contains programs to create and launch ROS bagfiles using the data provided in the [Robot@Home dataset](http://mapir.isa.uma.es/mapirwebsite/index.php/mapir-downloads/203-robot-at-home-dataset.html) (generated by the MAPIR group from Universidad de Malaga). 
Separate bagfiles are created from laser and each RGB-D camera data.

## 1. Installation 

1. This package works in **ROS melodic** (and **kinetic**). For instructions on how to install and configure a ROS workspace see
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

2. Copy the folder of 'robotathome_at_ros' package into the src folder. It should look like this:  
```
    ROS_WORKSPACE_PATH/
        build/
        devel/
        src/
          robotathome_at_ros/
```

3. To compile the package, just type catkin_make inside the main folder of the workspace.  
    `catkin_make`

## 2. Creating bagfiles from the data provided in the [Robot@Home dataset](http://mapir.isa.uma.es/mapirwebsite/index.php/mapir-downloads/203-robot-at-home-dataset.htmlurl)

The original Robot@Home dataset contains sensor data (e.g. laser, 4 rgbd cameras) of runs obtained in different rooms (e.g 'fullhouse1', 'livingroom1', 'bedroom1', 'kitchen1', 'bathroom1') from different houses (e.g 'alma-s1 ',' anto-s1 ',' pare-s1 ',' rx2-s1 '). They are available in a specific format used by the open source Mobile Robot Programming Toolkit (MRPT) or in (human readable) plain text files and png images. This ROS package generates bagfiles from the plain data (the latter type).

1. To generate the bagfile for a specific sensor for a given room in a house, type:  
    `rosrun robotathome_at_ros bag_scans.py PATH_TO_DATASET_FOLDER`

Obs: `PATH_TO_DATASET_FOLDER` is the internal directory of the selected run inside the unzipped folder that was downloaded from the Robot@Home webpage.

Ex: Assuming you want to generate the bagfile of the **laser** sensor information from the dataset **'alma-s1'**, room **'fullhouse1'**, you could type:  
    `rosrun robotathome_at_ros bag_scans.py ~/Downloads/Robot@Home-dataset_laser_scans-plain_text-alma-s1/alma-s1/fullhouse1/`

2. The resulting bagfile will be inside the ROS workspace in the folder:  
    `ROS_WORKSPACE_PATH/src/robotathome_at_ros/bagfiles/`

Ex:  
    `ROS_WORKSPACE_PATH/src/robotathome_at_ros/bagfiles/alma-s1/fullhouse1/alma-s1_fullhouse1_LASER-1_hokuyo_processed.bag`

## 3. Running multiple bagfiles of a given run with a launch file

In the 'launch' folder are available launch files to simultaneously run multiple bagfiles from the same run.

- Running 2D Laser + Frontal RGB-D Camera:  
    `roslaunch robotathome_at_ros run_single_cam_data.launch`

- Running 2D Laser + All 4 RGB-D Cameras:  
    `roslaunch robotathome_at_ros run_multi_cam_data.launch`

OBS: In addition to running the bagfiles, both launch files also run other things, such as: point cloud generators, RViz (for visualization), hector_slam (for generating a 2D map from the laser data and estimating the robot pose).
- If hector_slam is not installed, type:  
    `sudo apt install ros-VERSION-hector-slam`

## 4. Generating 2D Maps for Localization methods

This package also contains scripts to generate 2D maps used in the **FSD localization** approach (https://github.com/phir2-lab/fsd_localization). The scripts are in the 'maps' folder.

From bitmap images of 2D maps generated by a SLAM method (using laser or depth information), in a specific resolution (e.g. 1 pixel - cell of 5cm):

1. Manually create a simplified map of the walls (blueprint) in Inkscape (.svg format) and save clean version as HTML 5 Canvas (.html)

2. Converts from html to .map (which is map format used in MobileSim software - it is not necessary to have the software installed), informing the scale of the cells in millimeters (e.g. 50 millimeters per cell)  
    `python html2mobilesim.py <mapName>.html <cellsize in mm>`

The output is automatically set to `<mapName>.map`

3. Convert from the .map format to a 2D grid in txt format  
    `python generateWallsfromMap.py -i <inputfile> -o <outputfile> -s <cellsize in mm>`

Ex:
    1. Manually generate 'alma.html' with Inkscape  
    2. `python ../html2mobilesim.py alma.html 50`  
    3. `python ../generateWallsfromMap.py -i alma.map -o fullhouse1.txt -s 50`  

OBS: All map files for the 'alma-s1', 'anto-s1', 'pare-s1' and 'rx2-s1' datasets are already generated and available in the 'maps' folder