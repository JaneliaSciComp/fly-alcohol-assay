#!/usr/bin/python
from __future__ import print_function
# import argparse
import os
from diskwalk import DiskWalk
from file_tools import FileTools

FILE_TOOLS = FileTools()


class FindData(object):
    def __init__(self,overwrite=False):
        self.overwrite = overwrite

    def condition_path(self,path):
        return os.path.normpath(os.path.expanduser(path))

    def find_tracking_data(self,directory):
        directory = self.condition_path(directory)
        dw = DiskWalk(directory)
        paths_tracking_ext = dw.enumerate_paths_with_ext(FILE_TOOLS.tracking_ext)
        tracking_paths = []
        for path in paths_tracking_ext:
            (dir,file) = os.path.split(path)
            if file.startswith(FILE_TOOLS.tracking_prefix):
                dw_path = DiskWalk(dir)
                tracking_files_in_dir = dw_path.enumerate_paths_with_ext(FILE_TOOLS.tracking_ext)
                # one tracking file should already exist, more are produced when analyzed
                if self.overwrite or len(tracking_files_in_dir) == 1:
                    tracking_paths.append(path)
                # else:
                #     print(path + " tracking data already processed!")
        return tracking_paths

    def find_video_data(self,directory):
        directory = self.condition_path(directory)
        dw = DiskWalk(directory)
        paths_bag_ext = dw.enumerate_paths_with_ext(FILE_TOOLS.bag_ext)
        video_paths = []
        for path in paths_bag_ext:
            (dir,file) = os.path.split(path)
            if file.startswith(FILE_TOOLS.bag_prefix):
                dw_path = DiskWalk(dir)
                video_files_in_dir = dw_path.enumerate_paths_with_ext(FILE_TOOLS.video_ext)
                if self.overwrite or len(video_files_in_dir) == 0:
                    video_paths.append(path)
                # else:
                #     print(path + " bag data already processed!")
        return video_paths

    def find_figure_data(self,directory):
        directory = self.condition_path(directory)
        dw = DiskWalk(directory)
        paths_bag_ext = dw.enumerate_paths_with_ext(FILE_TOOLS.bag_ext)
        figure_paths = []
        for path in paths_bag_ext:
            (dir,file) = os.path.split(path)
            if file.startswith(FILE_TOOLS.bag_prefix):
                dw_path = DiskWalk(dir)
                figure_files_in_dir = dw_path.enumerate_paths_with_ext(FILE_TOOLS.figure_ext)
                tracking_files_in_dir = dw_path.enumerate_paths_with_ext(FILE_TOOLS.tracking_ext)
                if tracking_files_in_dir > 1 and (self.overwrite or len(figure_files_in_dir) == 0):
                    figure_paths.append(path)
                # else:
                #     print(path + " tracking data already processed!")
        return figure_paths

    def path_contains_tracking_data(self,path):
        path = self.condition_path(path)
        if not os.path.exists(path):
            return False
        tracking_paths = self.find_tracking_data(path)
        return (len(tracking_paths) > 0)

    def path_contains_video_data(self,path):
        path = self.condition_path(path)
        if not os.path.exists(path):
            return False
        video_paths = self.find_video_data(path)
        return (len(video_paths) > 0)

    def path_contains_figure_data(self,path):
        path = self.condition_path(path)
        if not os.path.exists(path):
            return False
        figure_paths = self.find_figure_data(path)
        return (len(figure_paths) > 0)

    # def path_contains_data(self,path):
    #     path = self.condition_path(path)
    #     if not os.path.exists(path):
    #         return False
    #     tracking_paths = self.find_tracking_data(path)
    #     video_paths = self.find_video_data(path)
    #     figure_paths = self.find_figure_data(path)
    #     return (len(tracking_paths) > 0) or (len(video_paths) > 0) or (len(figure_paths) > 0)


if __name__ == '__main__':
    pass
    # parser = argparse.ArgumentParser()
    # parser.add_argument("directory", help="Directory where tracking files are located")
    # parser.add_argument("-o", "--overwrite", dest='overwrite', default=False, action="store_true", help="Reanalyze all experiments and overwrite previous data (default is to only analyze new data)")
    # args = parser.parse_args()

    # dp = DataProcessor(overwrite=args.overwrite)
    # data_paths = dp.find_tracking_data(args.directory)
    # dp.process_data(data_paths)
