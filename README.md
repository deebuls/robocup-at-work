## Install Ubuntu
The repository and its related components have been tested under the following Ubuntu distributions:

- ROS Groovy: Ubuntu 12.04

If you do not have a Ubuntu distribution on your computer you can download it here

     http://www.ubuntu.com/download

## Git - Version Control
### Install Git Software
Install the Git core components and some additional GUI's for the version control:

     sudo apt-get install git-core gitg gitk

### Set Up Git
Now it's time to configure your settings. To do this you need to open a new Terminal. First you need to tell git your name, so that it can properly label the commits you make:

     git config --global user.name "Your Name Here"

Git also saves your email address into the commits you make. We use the email address to associate your commits with your GitHub account:

     git config --global user.email "your-email@youremail.com"


### GIT Tutorial
If you have never worked with git before, we recommend to go through the following basic git tutorial:

     http://excess.org/article/2008/07/ogre-git-tutorial/


## ROS - Robot Operating System
### Install ROS Fuerte
The repository has been tested successfully with the following ROS distributions. Use the link behind a ROS distribution to get to the particular ROS installation instructions.

- ROS Groovy - http://www.ros.org/wiki/groovy/Installation/Ubuntu

NOTE: Do not forget to update your .bashrc! 


### ROS Tutorials
If you have never worked with ROS before, we recommend to go through the beginner tutorials provided by ROS:

     http://www.ros.org/wiki/ROS/Tutorials

In order to understand at least the different core components of ROS, you have to start from tutorial 1 ("Installing and Configuring Your ROS Environment") till tutorial 7 ("Understanding ROS Services and Parameters"). 

## Set up a catkin workspace

    mkdir -p ~/catkin_ws/src; cd ~/catkin_ws/src
    catkin_init_workspace
    cd ..
    catkin_make
    
## Clone and compile the thrird party software
First of all you have to clone the repository.

    cd ~/catkin_ws/src;
    git clone git@github.com:b-it-bots/mas_common_robotics.git

Then go on with installing further external dependencies:
       
    cd ~/catkin_ws/src/mas_common_robotics
    ./repository.debs
    
    source ~/catkin_ws/devel/setup.bash

And finally compile the repository:

    cd ~/catkin_ws
    catkin_make
