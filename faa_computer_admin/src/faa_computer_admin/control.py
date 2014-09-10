#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('faa_computer_admin')
import rospy
import argparse
import subprocess

from faa_utilities import FindData
from faa_data_processing import TrackingDataProcessor
from faa_data_processing import VideoDataProcessor
from faa_data_processing import FigureDataProcessor


def process(path_list,overwrite,tracking,video,figure):
    """
    Process data
    """
    fd = FindData(overwrite)
    path = path_list[0]
    if not tracking and not video and not figure:
        tracking = True
        video = True
        figure = True
    if figure and not tracking:
        tracking = True
    if tracking:
        contains_data = fd.path_contains_tracking_data(path)
        if not contains_data and overwrite:
            print("Path does not exist or does not contain tracking data.")
        elif not contains_data and not overwrite:
            print("Path does not exist, does not contain tracking data, or tracking data has already been processed.")
            print("Try -o overwrite switch to reprocess data.")
        tdp = TrackingDataProcessor(overwrite)
        tdp.find_and_process_data(path)
    if video:
        contains_data = fd.path_contains_video_data(path)
        if not contains_data and overwrite:
            print("Path does not exist or does not contain video data.")
        elif not contains_data and not overwrite:
            print("Path does not exist, does not contain video data, or video data has already been processed.")
            print("Try -o overwrite switch to reprocess data.")
        vdp = VideoDataProcessor(overwrite)
        vdp.find_and_process_data(path)
    if figure:
        contains_data = fd.path_contains_figure_data(path)
        if not contains_data and overwrite:
            print("Path does not exist or does not contain figure data.")
        elif not contains_data and not overwrite:
            print("Path does not exist, does not contain figure data, or figure data has already been processed.")
            print("Try -o overwrite switch to reprocess data.")
        fdp = FigureDataProcessor(overwrite)
        fdp.find_and_process_data(path)

def calibrate():
    """
    Starts the camera calibration application
    """
    _roslaunch('calibrate_camera.launch')

def experiment(no_usb_hardware):
    """
    Starts the experiment mode application
    """
    if no_usb_hardware:
        print("Running in test mode with no USB hardware attached.")
        options = {'hardware': "false"}
    else:
        print("USB hardware attached!")
        options = {'hardware': "true"}

    reuse_background_images = False
    if reuse_background_images:
        print("Reusing background images.")
        options['reusing_bg_images'] = "true"
    else:
        options['reusing_bg_images'] = "false"

    _roslaunch('experiment.launch',options)

def manual(no_usb_hardware):
    """
    Starts the manual mode application
    """
    if no_usb_hardware:
        print("Running in test mode with no USB hardware attached.")
        options = {'hardware': "false"}
    else:
        print("USB hardware attached!")
        options = {'hardware': "true"}
    _roslaunch('manual_control.launch',options)

def save_images():
    """
    Starts the save images application
    """
    options = {}
    _roslaunch('save_images.launch',options)

def _roslaunch(launch_file,options={}):
    """
    Runs a roslaunch file.
    """
    try:
        call_list = ['roslaunch', 'faa_launch', launch_file]
        for option in options:
            call_list.append(option + ":=" + str(options[option]))
        subprocess.call(call_list)
    except KeyboardInterrupt:
        return

def cli():
    parser = argparse.ArgumentParser(description='Fly Alcohol Assay Control')

    # parser.add_argument('-t','--test',action="store_true",
    #                     help='launch test.launch')
    parser.add_argument('-c','--calibrate',action="store_true",
                        help='launch calibrate_camera.launch')
    parser.add_argument('-e','--experiment',action="store_true", default=True,
                        help='launch experiment.launch')
    parser.add_argument('-n','--no-usb-hardware',action="store_true",
                        help='set testing mode when USB hardware is not attached')
    parser.add_argument('-p','--process',dest='process_path',nargs=1,default=False,
                        help='process data within directory')
    parser.add_argument('-o','--overwrite',action="store_true", default=False,
                        help='reprocess data and overwrite processed data files')
    parser.add_argument('-t','--tracking',action="store_true", default=False,
                        help='process tracking data')
    parser.add_argument('-v','--video',action="store_true", default=False,
                        help='process videos')
    parser.add_argument('-f','--figure',action="store_true", default=False,
                        help='process data figure')
    # parser.add_argument('-r','--reuse-background-images',action="store_true",
    #                     help='reuse background images when testing')
    parser.add_argument('-m','--manual',action="store_true",
                        help='launch manual control GUI')

    args = parser.parse_args()

    if args.process_path:
        process(args.process_path,args.overwrite,args.tracking,args.video,args.figure)
    elif args.calibrate:
        calibrate()
    # elif args.save_images:
    #     save_images()
    elif args.manual:
        manual(args.no_usb_hardware)
    elif args.experiment:
        experiment(args.no_usb_hardware)
