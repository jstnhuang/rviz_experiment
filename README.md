# rviz_experiment

Scripts, config files, and launch files for an RViz observational experiment.

## Installation
You will need to download [rviz_camera_publisher](https://github.com/jstnhuang/rviz_camera_publisher) to your catkin workspace, and to build it.

There is a ROS node wrapper around the [Opengazer](http://www.inference.phy.cam.ac.uk/opengazer/) eye tracker. I included a binary built for Ubuntu 12.04 64-bit. The reason why it's in the repository is that the eye tracker didn't build correctly out of the box. I added instructions from the Ubuntu forums that fixes the build to the README in the opengazer directory, if you want to rebuild it. You will need to install all of its prerequisites, anyway.

## Procedure
### To run an experiment
Run interactive manipulation on the robot: `roslaunch pr2_interactive_manipulation pr2_interactive_manipulation_robot.launch`

1. Decide on a participant ID (`scripts/new_participant.py` will generate a random one).
1. On your computer, set ROS_MASTER_URI appropriately.
2. Run `scripts/reset_robot.sh`.
3. Run `scripts/experiment.sh 123 acclimation`, where 123 is the participant ID, and acclimation is the condition (either demonstration, acclimation, or experiment).
4. The eye tracker will start. To use the eye tracker, click on the corners of your eyes, then click on some "salient" points on your face. Finally, click "Calibrate." After the calibration process, the gaze position will be published to a topic.
5. Start publishing the rviz camera position by clicking the "Publish camera pose" button.

Experiment data will be saved to /home/jstn/Dropbox/experiment_data/123_acclimation.bag. Obviously, you should change the directory for your machine, in the experiment.launch and playback.launch files.

### To play back an experiment
1. Run `scripts/reset_robot.sh`
1. Run `scripts/playback.sh 123 experiment`.
2. `scripts/process_data.py <bag_file>` will compute some stats about the experiment data.
