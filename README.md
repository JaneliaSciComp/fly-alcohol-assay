#fly-alcohol-assay

The code to control the Janelia fly alcohol assay (FlyBar).

Authors:
Peter Polidoro polidorop@janelia.hhmi.org


##Running

###Experiment Mode (Typical Case)

Open a terminal and run::

    faa

Then open a web browser and navigate to http://localhost:5000


###Manual Mode for Adjusting Gates

Open a terminal and run::

    faa -m

###Help to Find More Command Line Options

Open a terminal and run::

    faa -h



##Installation

###Computer Hardware

* ASUS BP6375-I73770039B No Screen Desktop PC Intel Core i7
  3770(3.40GHz) 8GB DDR3 1TB HDD Capacity Intel HD Graphics 4000
  newegg item # N82E16883220285

* SIIG 3-Port FireWire PCI Express Card Model NN-E20022-S1. Newegg
  newegg item # N82E16815150160

* SAMSUNG 840 Series MZ-7TD500KW 2.5" 500GB SATA III Internal Solid
  State Drive (SSD)
  newegg item # N82E16820147187 (Quantity 2)

###Computer Operating System

xubuntu-12.04.3-desktop-amd64
md5 hash: 8719cc129fcb766d728a7c101eb6cec4

###Setup

Open a terminal and run::

    sudo apt-get update
    sudo apt-get upgrade

Open a terminal and run::

    sudo apt-get install git -y
    sudo apt-get install gparted -y
    sudo gparted

Format second internal harddrive to ext4 with label "data"

Open a terminal and run::

    sudo blkid
    # check to make sure harddrive device shows up with LABEL="data"
    sudo mkdir /media/data
    sudo emacs /etc/fstab

Add the following line to fstab::

    LABEL=data  /media/data  ext4  rw,suid,dev,noexec,auto,nouser,sync  0  0

Open a terminal and run::

    sudo shutdown -r now

After the computer restarts, open a terminal and run::

    sudo chown -R fly:fly /media/data
    ln -s /media/data ~/faa_data

###Setup for Remote Maintenance

On the machine you will use for remote maintenance, open a terminal
and run::

    sudo apt-get install vinagre
    ssh-keygen
    ssh-copy-id [remote login name]@[IP address of remote computer]

On the remote computer open a terminal and run::

    sudo apt-get install x11vnc
    sudo emacs /etc/ssh/sshd_config

Make sure PubkeyAuthentication is set to yes and
PasswordAuthentication is set to no.

###Test the Camera

Open a terminal and run::

    sudo apt-get install coriander -y
    coriander
    # test to make sure camera works properly

###Setup Permissions to Connect to Arduino Devices

Open a terminal and run::

    sudo usermod -aG dialout $USER
    sudo shutdown -r now

###Install ROS

Open a terminal and run::

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
    wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
    sudo apt-get update
    sudo apt-get dist-upgrade
    sudo apt-get install ros-groovy-desktop-full -y

###Create ROS Workspace and Install Repositories

Open a terminal and run::

    sudo apt-get install python-rosinstall -y
    mkdir ~/ros
    rosws init ~/ros/faa_ws /opt/ros/groovy
    source ~/ros/faa_ws/setup.bash
    rosws set faa --git https://github.com/janelia-idf/fly-alcohol-assay.git
    rosws set faa_config --git https://github.com/janelia-idf/fly-alcohol-assay-config.git
    rosws update
    python ~/ros/faa_ws/fly-alchol-assay-config/bash_setup.py
    source ~/.bashrc
    python ~/ros/faa_ws/fly-alcohol-assay/install.py

###Install Extra ROS Pacakges (manually for now, automatically in the future)

Open a terminal and run::

    sudo apt-get install ros-groovy-camera1394 -y
    sudo apt-get install ros-groovy-mjpeg-server -y
    sudo apt-get install ros-groovy-rosbridge-suite -y
    sudo apt-get install ros-groovy-executive-smach-visualization -y

###Install Support Python Packages into Virtualenv (manually for now, automatically in the future)

Open a terminal and run::

    source $FAA_PYTHON_VIRTUALENV/bin/activate
    pip install flask --upgrade

###Install Codecs for Creating and Viewing Videos

Open a terminal and run::

    sudo apt-get install libavcodec-extra-53 vlc

##Setup

###Comiple the ROS Packages

Open a terminal and run::

    rosmake faa

###Calibration

The path to the checkerboard pattern is:
~/ros/faa_ws/faa_config/patterns/checkerboard_10mm.svg

Print out at 100% scale using inkscape. Check to make sure each small
black square has side length 10mm. Place double-sided tape on back
side of printout and then cut out around outer black square
borders. Tape onto a 6"x6" flat plate and use roller to press on
evenly.

Open a terminal and run::

    faa -c

Use 8x6 10mm checkerboard to calibrate the camera.  Remove IR filter
on camera if necessary and provide enough visible light for camera to
see the checkerboard. Wave checkerboard around under camera until
'Calibrate' button activates.  Press 'Calibrate' button, then press
'Commit'. Place IR filter back onto camera.


##Software Update

Open a terminal and run::

    cd ~/ros/faa_ws/faa
    git pull origin master
    rosmake faa
