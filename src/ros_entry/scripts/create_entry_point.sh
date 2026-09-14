#!/usr/bin/env zsh
source /home/mkac/.zshrc
cd $(rospack find ros_entry)/scripts
sudo cp amr_simulation.desktop /usr/share/applications/amr_simulation.desktop
cp amr_desktop_entry.desktop $HOME/Desktop/amr_desktop_entry.desktop
cp hmi_desktop_entry.desktop $HOME/Desktop/hmi_desktop_entry.desktop
# Set robot config. Be careful because the old file will be overwritten
# mkdir -p $HOME/robot_config
# cp $(rospack find ros_entry)/cfg/ros_config.sh $HOME/robot_config/ros_config.sh
# cp $(rospack find ros_entry)/cfg/robot_define.yaml $HOME/robot_config/robot_define.yaml
# TODO: Set icon
sudo cp $(rospack find amr_config)/media/amr.png /usr/share/pixmaps/amr.png
# Make auto start
sudo cp amr_auto_start.desktop $HOME/.config/autostart/amr_auto_start.desktop