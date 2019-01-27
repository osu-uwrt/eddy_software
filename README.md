## The Eddy AUV Software Platform   

This repository is the secondary code base for the Underwater Robotics Team at The Ohio State University. Our mission is to develop the software which powers our Autonomous Underwater Vehicles. The software in this repository is designed for our test vehicle, Eddy, and is written purely by our newest members to learn basic data processing and controls algorithms. All built on the [Robot Operating System](http://www.ros.org/) framework.



**The Underwater Robotics Team**  
The Ohio State University

[Website](https://uwrt.engineering.osu.edu) | [RoboSub](https://www.auvsifoundation.org/competition/robosub) | [License](LICENSE)


![OSU UWRT Logo](https://github.com/osu-uwrt/riptide_software/blob/master/logos/UWRT_Logo_small.png)

# Building
ROS is compiled using the catkin build system, and so this repo will use catkin. 

## Cloning
To collaborate with the eddy_software platform, you must fork this repo (click "Fork" at the top-right of this page). When executing the commands below, you will need to enter the URL to your forked repo. Form YOUR forked repo, click "Clone or download" at the top-right of the page, copy the URL, and then insert that URL in place of "<your_forked_repo>". Do NOT forget the "src" at the end of the last line. This is a catkin-specific requirement that all source code be placed within a folder called "src".
```
mkdir -p ~/osu-uwrt/eddy_software/
cd ~/osu-uwrt/eddy_software/
git clone <your_forked_repo> src
```

## Setting up Git Remotes
Since you just cloned your fork to your computer, your remote called "origin" will point to your fork. Now, create a new remote that points to this main repo.
```
cd ~/osu-uwrt/eddy_software/src/
git remote add upstream https://github.com/osu-uwrt/eddy_software.git
```

Now, if you type:
```
git remote -v
```
You will see both a remote to your fork and to the main repo. You will use these two remotes a lot when pushing code to your fork, submitting pull-requests, and pulling down the latest code.

## Compiling
To compile this repo, you simply execute the "catkin_make" command from a terminal. As a word of caution, you MUST be inside the folder "~/osu-uwrt/eddy_software" to run "catkin_make"
```
cd ~/osu-uwrt/eddy_software/
catkin_make
```

### Updating the ~/.bashrc File
Because this repo is built on ROS, there are a number of environment variables that are required when running the code. Each time you open a terminal window to run those commands, those environment variables will need to be set. To automate this process, we add a line to the "bashrc file" (bashrc = Born Again Shell Run-Commands). Each time a new terminal/shell is opened, it executes any commands within the "bashrc" file. This is a hidden file that lies within the home directory (hence the "~/.").

Open it with a terminal text editor:
```
nano ~/.bashrc
```
Scroll down to the bottom. Beneath the line
```
source /opt/ros/kinetic/setup.bash
```
add the line:
```
source ~/osu-uwrt/eddy_software/devel/setup.bash
```
If you already have several lines sourcing the workspaces for "riptide_software," then place the line directly beneath:
```
source ~/osu-uwrt/riptide_software/devel/setup.bash
```
To exit and save your changes, press CTRL-X, type "y", then press ENTER. For these changes to take effect in the CURRENT terminal window we have to "source" it because the terminal was opened a while ago. Run the command:
```
source ~/.bashrc
```

You could also just close the terminal and open a new one, because opening a terminal will automatically run the "~/.bashrc" file.
