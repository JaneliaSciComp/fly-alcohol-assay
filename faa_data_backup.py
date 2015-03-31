#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 25 20:08:54 2015

@author: Nicholas Mei

Simple convenience script that automates the following processes:

Partial Backup (For fast data analysis and transfer)
A1) Run "faa -p ~/faa_data_directory -t -o" on data in a experiment day folder
A2) Transfer of folder directory onto mapped network drive (in our case the Synology server)

Full Backup (Go through the full analysis, bag compression, and transfer)
B1) Running "faa -p ~/faa_data/directory -f -o" on data in a folder
B2) Running of "rosbag compress ~/faa_data/directory/*/*/*.bag" on data for a chosen day
B3) Transfer of chosen folder directory onto mapped network drive while ignoring uncompressed *.orig.bag files

NOTE: Make sure to change the "default_choose_dir" and "default_save_dir" variables
found in the main() function as your backup directory and data directory will
inevitably be different!
"""

import os
import Tkinter, tkFileDialog
import shutil
import errno
import shlex
import glob
import argparse

import subprocess as sp

def faa_copy(src, dest, ignore_patt=None):
    try:
        shutil.copytree(src, dest, ignore=shutil.ignore_patterns(ignore_patt))
    except OSError as e:
        # If the error was caused because the source wasn't a directory
        if e.errno == errno.ENOTDIR:
            print('User chose a file not a directory!')
        else:
            print('Directory not copied. Error: %s' % e)

def faa_backup(src_dir, dest_dir, full_backup, overwrite = False):
    final_dest_exists = False
    
    #First import OS environment variables since we'll always need them
    sp_env = os.environ.copy()
    
    final_dest = os.path.join(dest_dir, os.path.basename(src_dir))
    #Then check if the final destination already exists and delete it if it does
    if os.path.isdir(final_dest):
        if overwrite is False:
            print "The final destination directory: {} already exists. In order to proceed, it must be deleted".format(final_dest)
            choice = raw_input("Proceed with deletion? Y/[N]\n").lower()
        
            if choice in ['yes', 'y', 'ye']:
                try:
                    shutil.rmtree(final_dest)
                except OSError as e:
                    print('Directory could not be deleted. Error: %s' % e)
            else:
                final_dest_exists = True
                print "Cannot continue because {} already exists and you chose not to delete it!".format(final_dest)
        else:
            print "Overwrite specified, deleting and remaking: {}".format(final_dest)
            try:
                shutil.rmtree(final_dest)
            except OSError as e:
                print('Directory could not be deleted. Error: %s' % e)
            
    #Check which type of backup 
    if (full_backup is True) and (final_dest_exists is False):
        #B1) First step is running "faa -p ~/faa_data/directory -f -o" on data in a folder
        faa_command = "/bin/bash faa -p {}/ -f -o".format(src_dir)
        faa_command = shlex.split(faa_command)
        faa_status = sp.call(faa_command, env=sp_env)
    
        #Check if the faa program ended without any errors (status == 0)
        if faa_status is 0:
            modified_src_path = src_dir + "/*/*/*.bag"
            files_to_compress = glob.glob(modified_src_path)       
        
            #B2) Second step, running "rosbag compress ~/faa_data/directory/*/*/*.bag"
            rosbag_command = "rosbag compress {}".format(' '.join(files_to_compress))
            rosbag_command = shlex.split(rosbag_command)        
            rosbag_status = sp.call(rosbag_command, env=sp_env)
        
            #Check if rosbag program ended without any errors
            if rosbag_status is 0:
                #B3) Final step, copy folder directory over to destination
                faa_copy(src_dir, final_dest, ignore_patt='*orig.bag')
            #Rosbag status fail
            else:
                print "Rosbag did not complete successfully!!"           
        #FAA status fail 
        else:
            print "FAA analysis did not complete successfully!"
        
    #In the case when we're not doing full_backup
    elif (final_dest_exists is False):        
        #A1) First step is running 'faa -p ~/faa_data/directory -t -o" on data in folder
        faa_command = "/bin/bash faa -p {}/ -t -o".format(src_dir)
        faa_command = shlex.split(faa_command)
        faa_status = sp.call(faa_command, env=sp_env)

        #Check if faa program ended without any errors
        if faa_status is 0:
            #A2) Final step, copy folder directory over to destination
            faa_copy(src_dir, final_dest, ignore_patt='*.bag')
        #FAA status fail
        else:
            print "FAA analysis did not complete successfully!"

def main():

    #Let's parse the arguments provided to the program
    parser = argparse.ArgumentParser(description = 'Backup FAA data to a destination directory')
    backup_type = parser.add_mutually_exclusive_group(required=True)
    backup_type.add_argument('-p', '-partial', '--partial', action='store_false', default=False, dest= 'full_backup', help='Perform a partial (quick) backup by running "faa -p -t" and then transferring')
    backup_type.add_argument('-f', '-full', '--full', action='store_true', default=False, dest= 'full_backup', help='Perform a full backup by running "faa -p -f", "rosbag compress", and then transferring')
    parser.add_argument('-s', '-source', '--source', dest='src_dir', nargs = '?', help='Specify in arguments the source FAA data directory to backup. Leaving blank will instead call up an ask directory dialogue')
    parser.add_argument('-d', '-dest', '--dest', dest='dest_dir', nargs = '?', help='Specify in arguments the destination directory to backup FAA data to. Leaving blank will instead call up an ask directory dialogue')
    parser.add_argument('-o', '-overwrite', '--overwrite', action='store_true', dest='overwrite', default=False, help='Include this option to overwrite existing src data in the destination location by default')

    args = parser.parse_args()
    #print args

    #Check if the src_dir or dest_dir arguments exist. If not get them!
    if (args.src_dir is None) or (args.dest_dir is None):

        root = Tkinter.Tk()
        root.withdraw()

        if args.src_dir is None:
            default_choose_dir = "~/faa_data"
            args.src_dir =  tkFileDialog.askdirectory(parent=root, title='Choose experiment day to run initial analysis and backup on', initialdir=default_choose_dir)

        if args.dest_dir is None:
            default_save_dir = "/home/fly/Synology/Nick's EtOH flybar dose experiments"
            args.dest_dir = tkFileDialog.askdirectory(parent=root, title='Choose a directory to backup analysis to', initialdir=default_save_dir)
            
    #Check if the acquired source and destination directories actually exist!
    if (os.path.isdir(args.src_dir)):
        if (os.path.isdir(args.dest_dir)):
            print '\n\nOkay! You chose the following directories:\nSource Directory: {}\nDestination Directory: {}\nFull Backup?: {}'.format(args.src_dir, args.dest_dir, str(args.full_backup))
            choice = raw_input("Is this information correct? Y/[N]\n").lower()
            if choice in ['yes', 'y', 'ye']:
                print "Great! Starting backup!"
                faa_backup(args.src_dir, args.dest_dir, args.full_backup, overwrite = args.overwrite)
            else:
                print 'Provide the correct information and try again!'                    
        else:
            print 'Error! The destination directory you specified: {} Does not exist! Check the destination path and try again!'.format(args.dest_dir)
    else:
        print 'Error! The source directory you specified: {} Does not exist! Check the source path and try again!'.format(args.src_dir)
                
if __name__ == "__main__":
    main()
