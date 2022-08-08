# HABApp-ROS-openHAB-Bridge
A bridge between openHAB and ROS using HABApp

## Preparations

You have to make sure that ROS, openHAB and HABApp are installed on one and the same system.

### Install openHAB on Ubuntu Linux

At first you have to install `Java`:

```
# install the necessary dependencies
sudo apt-get -q update
sudo apt-get -yq install gnupg curl 

# add Azul's public key
sudo apt-key adv \
  --keyserver hkp://keyserver.ubuntu.com:80 \
  --recv-keys 0xB1998361219BD9C9

# download and install the package that adds 
# the Azul APT repository to the list of sources 
curl -O https://cdn.azul.com/zulu/bin/zulu-repo_1.0.0-3_all.deb

# install the package
sudo apt-get install ./zulu-repo_1.0.0-3_all.deb

# update the package sources
sudo apt-get update

# install Azul Zulu JDK 11
sudo apt-get install zulu11-jdk
```

After that you have to install `openHAB 3` and its addons:

```
curl -fsSL "https://openhab.jfrog.io/artifactory/api/gpg/key/public" | gpg --dearmor > openhab.gpg
sudo mkdir /usr/share/keyrings
sudo mv openhab.gpg /usr/share/keyrings
sudo chmod u=rw,g=r,o=r /usr/share/keyrings/openhab.gpg

echo 'deb [signed-by=/usr/share/keyrings/openhab.gpg] https://openhab.jfrog.io/artifactory/openhab-linuxpkg stable main' | sudo tee /etc/apt/sources.list.d/openhab.list

sudo apt-get update
sudo apt-get install openhab
sudo apt-get install openhab-addons
```

To run openHAB automaticall after restart you have to enable it:

```
sudo systemctl enable openhab.service
```

### Install HABApp on Ubuntu Linux

If you are using an older Ubuntu Linux distribution please make sure that you install `python3`, `pip3` and `python3-virtualenv`.

You have to install `HABApp` like this:

```
cd /opt
python3 -m venv habapp
cd habapp
source bin/activate
python3 -m pip install --upgrade pip setuptools
python3 -m pip install habapp

habapp --config /etc/openhab/habapp
```

To make `HABApp` run after reboot you have to create a service file with `sudo nano /etc/systemd/system/habapp.service`:

```
[Unit]
Description=HABApp
Documentation=https://habapp.readthedocs.io
After=network-online.target

[Service]
Type=simple
User=openhab
Group=openhab
UMask=002
ExecStart=/opt/habapp/bin/habapp -c /etc/openhab/habapp

[Install]
WantedBy=multi-user.target
```

And then enable it with:

```
sudo systemctl enable habapp.service
```

### Install ROS 1 on Ubuntu Linux

[ROS Noetic for Ubuntu 20.04](http://wiki.ros.org/noetic/Installation/Ubuntu)

[ROS Melodic for Ubuntu 18.04](http://wiki.ros.org/melodic/Installation/Ubuntu)

[ROS Kinetic for Ubuntu 16.04](http://wiki.ros.org/kinetic/Installation/Ubuntu)

After installing ROS you have to create a `catkin workspace`:


```
source /opt/ros/<distro>/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

As example for ROS Kinetic:

```
source /opt/ros/kinetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

## Installation

### Hack HABApp for using ROS

Further informations you can get in the [openHAB community](https://community.openhab.org/t/hacked-habapp-for-using-ros/135318).

Edit `/opt/habapp/bin/habapp` to:

```
#!/opt/habapp/bin/python3
# -*- coding: utf-8 -*-
import re
import sys
import rospy
from HABApp.__main__ import main
if __name__ == '__main__':
    rospy.init_node("habapp", anonymous=False)
    sys.argv[0] = re.sub(r'(-script\.pyw|\.exe)?$', '', sys.argv[0])
    sys.exit(main())
```

So after starting `HABApp` you should see the node `habapp`. You could also change the node name to `openhab` then you will see of course the node `openhab`:

```
rosnode list
/habapp
/rosout
```

### Install the openhab_msgs inside your catkin workspace

In the next step you have to install the openhab_msgs in your catkin workspace. For further informations you have to look in the [openHAB community](https://community.openhab.org/t/creating-ros-openhab-bridge-with-habapp-ros-openhab-msgs-openhab-subscribers-and-openhab-publishers/135406).

```
cd ~/catkin_ws/src
git clone https://github.com/Michdo93/openhab_msgs
cd ~/catkin_ws
catkin_make
```

The openhab_msgs creates custom ROS message types for each [openHAB item state or command type](https://www.openhab.org/docs/configuration/items.html#type).

### Install the ROS openHAB Bridge

You have to go to the rules folder of HABApp and download the python file. After that you have to make it executable:

```
cd /etc/openhab/habapp/rules
wget https://raw.githubusercontent.com/Michdo93/HABApp-ROS-openHAB-Bridge/main/openhab_bridge.py
sudo chown +x openhab_bridge.py
```

Please make sure that in `/etc/openhab/habapp/config.yml` the connection is set to your openHAB instance!

### Install the openHAB Bridge Publisher and Subscriber for testing

The [openHAB Bridge Publisher](https://github.com/Michdo93/openhab_bridge_publisher) publishes commands to openhab using the bridge between openHAB and ROS.
The [openHAB Bridge Subscriber](https://github.com/Michdo93/openhab_bridge_subscriber) subscribes states from openhab using a bridge between openHAB and ROS.

```
cd ~/catkin_ws/src
git clone https://github.com/Michdo93/openhab_bridge_publisher
git clone https://github.com/Michdo93/openhab_bridge_subscriber
cd ~/catkin_ws
catkin_make
```

In a real scenario you have to run ROS over [multiple machines](http://wiki.ros.org/ROS/Tutorials/MultipleMachines). As example one machine is your `openHAB Server` and another machine is your robot.
