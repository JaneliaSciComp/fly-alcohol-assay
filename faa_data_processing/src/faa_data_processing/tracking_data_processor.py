#!/usr/bin/python
from __future__ import print_function
import roslib; roslib.load_manifest('faa_data_processing')
import rospy

import argparse
import os
import numpy
import numpy.lib.recfunctions as recfunctions
import csv
from faa_utilities import FindData
from faa_utilities import FileTools

FILE_TOOLS = FileTools()


class TrackingDataProcessor(object):
    def __init__(self,overwrite=False):
        self.overwrite = overwrite
        self.fd = FindData(overwrite=overwrite)

    def find_and_process_data(self,directory):
        paths = self.find_data(directory)
        self.process_data(paths)

    def find_data(self,directory):
        return self.fd.find_tracking_data(directory)

    def process_data(self,paths):
        for path in paths:
            print("Processing data in {0}".format(path))
            numpy_data = FILE_TOOLS.load_numpy_data(path)
            tracking_data = self.remove_pre_trial_data(numpy_data)
            # header: time_secs,time_nsecs,status,tunnel,enabled,gate0,gate1,gate2,fly_x,fly_y,fly_angle,chamber,blob_x,blob_y,blob_area,blob_slope,blob_ecc
            normalized_data = self.normalize_data(tracking_data)
            filtered_data = self.filter_data(normalized_data)

            raw_walkway_data = self.get_raw_walkway_data(filtered_data)
            raw_chamber_data = self.get_raw_chamber_data(filtered_data)
            self.write_data(path,FILE_TOOLS.raw_prefix+FILE_TOOLS.walkway_prefix,raw_walkway_data)
            self.write_data(path,FILE_TOOLS.raw_prefix+FILE_TOOLS.chamber_prefix,raw_chamber_data)

            analyzed_walkway_data = self.analyze_data(raw_walkway_data)
            self.write_data(path,FILE_TOOLS.analyzed_prefix+FILE_TOOLS.walkway_prefix,analyzed_walkway_data)
            summarized_walkway_data = self.summarize_data(analyzed_walkway_data)
            self.write_data(path,FILE_TOOLS.summarized_prefix+FILE_TOOLS.walkway_prefix,summarized_walkway_data)

            analyzed_chamber_data = self.analyze_chamber_data(raw_chamber_data)
            self.write_data(path,FILE_TOOLS.analyzed_prefix+FILE_TOOLS.chamber_prefix,analyzed_chamber_data)
            summarized_chamber_data = self.summarize_chamber_data(analyzed_chamber_data)
            self.write_data(path,FILE_TOOLS.summarized_prefix+FILE_TOOLS.chamber_prefix,summarized_chamber_data)

    def remove_pre_trial_data(self,tracking_data):
        indicies = tracking_data['status'] != 'Wait in Start'
        tracking_data = tracking_data[indicies]
        return tracking_data

    def normalize_data(self,tracking_data):
        time_secs = numpy.float64(tracking_data['time_secs'])
        time_secs -= time_secs[0]
        time_nsecs = numpy.float64(tracking_data['time_nsecs'])*10**(-9)
        time_nsecs -= time_nsecs[0]
        time_rel = time_secs + time_nsecs

        names = list(tracking_data.dtype.names)
        norm_data = tracking_data[names]
        norm_data = recfunctions.append_fields(norm_data,'time_rel',time_rel,dtypes=numpy.float64,usemask=False)

        # frame = numpy.uint32(tracking_data['frame'])
        # frame -= frame[0]

        # names = list(tracking_data.dtype.names)
        # names.remove('time_secs')
        # names.remove('time_nsecs')
        # names.remove('frame')
        # norm_data = tracking_data[names]

        # norm_data = recfunctions.append_fields(norm_data,'time',time,dtypes=numpy.float64,usemask=False)
        # norm_data = recfunctions.append_fields(norm_data,'frame',frame,dtypes=numpy.uint16,usemask=False)
        return norm_data

    def filter_data(self,normalized_data):
        tunnels = set(normalized_data['tunnel'])
        indicies = None
        for tunnel in tunnels:
            tunnel_data = normalized_data[normalized_data['tunnel']==tunnel]
            enabled = tunnel_data['enabled'] == 'True'
            if numpy.all(enabled):
                if indicies is not None:
                    indicies |= normalized_data['tunnel'] == tunnel
                else:
                    indicies = normalized_data['tunnel'] == tunnel
        indicies &= normalized_data['chamber'] != ''
        indicies &= normalized_data['blob_ecc'] != '0.0'
        filtered_data = normalized_data[indicies]
        return filtered_data

    def get_raw_walkway_data(self,filtered_data):
        # walkway_dtype = numpy.dtype([('time_secs', '<u4'),
        #                              ('time_nsecs', '<u4'),
        #                              ('time_rel', '<f4'),
        #                              ('tunnel', '<u2'),
        #                              ('fly_x', '<f4'),
        #                              ('fly_y', '<f4'),
        #                              ('fly_angle', '<f4'),
        #                              ])
        header = list(FILE_TOOLS.walkway_dtype.names)
        walkway_data = filtered_data[filtered_data['status'] == 'Walk To End']
        walkway_data = walkway_data[walkway_data['gate1'] != 'close']
        walkway_data = walkway_data[header]
        walkway_data = walkway_data.astype(FILE_TOOLS.walkway_dtype)
        walkway_data['tunnel'] = walkway_data['tunnel']+1
        return walkway_data

    def get_raw_chamber_data(self,filtered_data):
        # chamber_dtype = numpy.dtype([('time_secs', '<u4'),
        #                              ('time_nsecs', '<u4'),
        #                              ('time_rel', '<f4'),
        #                              ('status', '|S25'),
        #                              ('tunnel', '<u2'),
        #                              ('fly_x', '<f4'),
        #                              ('fly_y', '<f4'),
        #                              ('fly_angle', '<f4'),
        #                              ])
        header = list(FILE_TOOLS.chamber_dtype.names)
        tracking_chamber_data = filtered_data[filtered_data['status'] != 'Walk To End']
        tracking_chamber_data = tracking_chamber_data[header]
        tracking_chamber_data = tracking_chamber_data.astype(FILE_TOOLS.chamber_dtype)
        tracking_chamber_data['tunnel'] = tracking_chamber_data['tunnel']+1
        indicies = tracking_chamber_data['status'] == 'End Chamber Ethanol'
        raw_chamber_data_ethanol = tracking_chamber_data[indicies]
        raw_chamber_data_ethanol = recfunctions.drop_fields(raw_chamber_data_ethanol,
                                                            'status',
                                                            usemask=False)
        status_array = numpy.array(['Ethanol']*len(raw_chamber_data_ethanol),dtype='|S25')
        raw_chamber_data_ethanol = recfunctions.append_fields(raw_chamber_data_ethanol,
                                                              'status',
                                                              status_array,
                                                              dtypes='|S25',
                                                              usemask=False)
        raw_chamber_data = raw_chamber_data_ethanol

        ethanol_start_time = raw_chamber_data_ethanol['time_rel'][0]
        indicies = tracking_chamber_data['status'] == 'End Chamber Air'
        indicies &= tracking_chamber_data['time_rel'] < ethanol_start_time
        raw_chamber_data_air_before = tracking_chamber_data[indicies]
        raw_chamber_data_air_before = recfunctions.drop_fields(raw_chamber_data_air_before,
                                                               'status',
                                                               usemask=False)
        status_array = numpy.array(['AirBefore']*len(raw_chamber_data_air_before),dtype='|S25')
        raw_chamber_data_air_before = recfunctions.append_fields(raw_chamber_data_air_before,
                                                                 'status',
                                                                 status_array,
                                                                 dtypes='|S25',
                                                                 usemask=False)
        raw_chamber_data = recfunctions.stack_arrays((raw_chamber_data_air_before,raw_chamber_data),usemask=False)

        indicies = tracking_chamber_data['status'] == 'End Chamber Air'
        indicies &= tracking_chamber_data['time_rel'] > ethanol_start_time
        raw_chamber_data_air_after = tracking_chamber_data[indicies]
        raw_chamber_data_air_after = recfunctions.drop_fields(raw_chamber_data_air_after,
                                                               'status',
                                                               usemask=False)
        status_array = numpy.array(['AirAfter']*len(raw_chamber_data_air_after),dtype='|S25')
        raw_chamber_data_air_after = recfunctions.append_fields(raw_chamber_data_air_after,
                                                                 'status',
                                                                 status_array,
                                                                 dtypes='|S25',
                                                                 usemask=False)
        raw_chamber_data = recfunctions.stack_arrays((raw_chamber_data,raw_chamber_data_air_after),usemask=False)

        return raw_chamber_data

    def write_data_to_file(self,path,data):
        header = list(data.dtype.names)
        fid = open(path, "w")
        data_writer = csv.writer(fid, delimiter=",")
        data_writer.writerow(header)
        data_writer.writerows(data)
        fid.close()

    def write_data(self,path,prefix,data):
        (dir,file) = os.path.split(path)
        if file.startswith(FILE_TOOLS.tracking_prefix):
            data_file = file.replace(FILE_TOOLS.tracking_prefix,prefix)
            data_path = os.path.join(dir,data_file)
            if os.path.exists(data_path) and not self.overwrite:
                print("Data file already exists!")
                return
        else:
            return
        print("Writing new data file")

        data_file_name = file.replace(FILE_TOOLS.tracking_prefix,prefix)
        data_path = os.path.join(dir,data_file_name)
        self.write_data_to_file(data_path,data)

    def analyze_data(self,raw_data):
        initialized = False
        tunnels = set(raw_data['tunnel'])
        for tunnel in tunnels:
            tunnel_data_raw = raw_data[raw_data['tunnel']==tunnel]
            time_rel = tunnel_data_raw['time_rel']
            delta_time = numpy.diff(time_rel)
            tunnel_array = numpy.ones(len(delta_time),dtype=numpy.uint16)*tunnel
            tunnel_array.dtype = numpy.dtype([('tunnel','<u2')])
            tunnel_data_analyzed = tunnel_array

            fly_x = tunnel_data_raw['fly_x']
            delta_fly_x = numpy.diff(fly_x)

            fly_y = tunnel_data_raw['fly_y']
            delta_fly_y = numpy.diff(fly_y)

            distance = numpy.sqrt(numpy.square(delta_fly_x)+numpy.square(delta_fly_y))

            velocity = distance/delta_time

            fly_angle = tunnel_data_raw['fly_angle']
            delta_fly_angle = numpy.abs(numpy.diff(fly_angle))
            flipped = 180 - delta_fly_angle
            flipped_is_less = flipped < delta_fly_angle
            delta_fly_angle[flipped_is_less] = flipped[flipped_is_less]

            angular_velocity = delta_fly_angle/delta_time

            time_secs = tunnel_data_raw['time_secs'][:-1]
            time_nsecs = tunnel_data_raw['time_nsecs'][:-1]

            names = ['time_secs','time_nsecs']
            tunnel_data_seq = [time_secs,time_nsecs]
            tunnel_data_analyzed = recfunctions.append_fields(tunnel_data_analyzed,
                                                              names,
                                                              tunnel_data_seq,
                                                              dtypes=numpy.uint64,
                                                              usemask=False)
            names = ['delta_time','delta_fly_x','delta_fly_y','distance','velocity','delta_fly_angle','angular_velocity']
            tunnel_data_seq = [delta_time,delta_fly_x,delta_fly_y,distance,velocity,delta_fly_angle,angular_velocity]
            tunnel_data_analyzed = recfunctions.append_fields(tunnel_data_analyzed,
                                                              names,
                                                              tunnel_data_seq,
                                                              dtypes=numpy.float32,
                                                              usemask=False)
            if initialized:
                analyzed_data = recfunctions.stack_arrays((analyzed_data,tunnel_data_analyzed),usemask=False)
            else:
                analyzed_data = tunnel_data_analyzed
                initialized = True
        return analyzed_data

    def analyze_chamber_data(self,raw_chamber_data):
        ethanol_data = raw_chamber_data[raw_chamber_data['status']=='Ethanol']
        analyzed_ethanol_data = self.analyze_data(ethanol_data)
        status_array = numpy.array(['Ethanol']*len(analyzed_ethanol_data),dtype='|S25')
        analyzed_chamber_data = recfunctions.append_fields(analyzed_ethanol_data,
                                                           'status',
                                                           status_array,
                                                           dtypes='|S25',
                                                           usemask=False)

        air_before_data = raw_chamber_data[raw_chamber_data['status']=='AirBefore']
        analyzed_air_before_data = self.analyze_data(air_before_data)
        status_array = numpy.array(['AirBefore']*len(analyzed_air_before_data),dtype='|S25')
        analyzed_air_before_data = recfunctions.append_fields(analyzed_air_before_data,
                                                              'status',
                                                              status_array,
                                                              dtypes='|S25',
                                                              usemask=False)
        analyzed_chamber_data = recfunctions.stack_arrays((analyzed_air_before_data,analyzed_chamber_data),usemask=False)


        air_after_data = raw_chamber_data[raw_chamber_data['status']=='AirAfter']
        analyzed_air_after_data = self.analyze_data(air_after_data)
        status_array = numpy.array(['AirAfter']*len(analyzed_air_after_data),dtype='|S25')
        analyzed_air_after_data = recfunctions.append_fields(analyzed_air_after_data,
                                                              'status',
                                                              status_array,
                                                              dtypes='|S25',
                                                              usemask=False)
        analyzed_chamber_data = recfunctions.stack_arrays((analyzed_chamber_data,analyzed_air_after_data),usemask=False)

        return analyzed_chamber_data

    def summarize_data(self,analyzed_data):
        initialized = False
        tunnels = set(analyzed_data['tunnel'])
        for tunnel in tunnels:
            tunnel_data_analyzed = analyzed_data[analyzed_data['tunnel']==tunnel]

            tunnel_array = numpy.ones(1,dtype=numpy.uint16)*tunnel
            tunnel_array.dtype = numpy.dtype([('tunnel','<u2')])
            tunnel_data_summarized = tunnel_array

            delta_time = tunnel_data_analyzed['delta_time']
            total_time = delta_time.sum()
            distance = tunnel_data_analyzed['distance']
            total_distance = distance.sum()
            velocity = tunnel_data_analyzed['velocity']
            mean_velocity = velocity.mean()
            angular_velocity = tunnel_data_analyzed['angular_velocity']
            mean_angular_velocity = angular_velocity.mean()

            names = ['total_time','total_distance','mean_velocity','mean_angular_velocity']
            tunnel_data_seq = [total_time,total_distance,mean_velocity,mean_angular_velocity]
            tunnel_data_summarized = recfunctions.append_fields(tunnel_data_summarized,
                                                                names,
                                                                tunnel_data_seq,
                                                                dtypes=numpy.float32,
                                                                usemask=False)
            if initialized:
                summarized_data = recfunctions.stack_arrays((summarized_data,tunnel_data_summarized),usemask=False)
            else:
                summarized_data = tunnel_data_summarized
                initialized = True

        return summarized_data

    def summarize_chamber_data(self,analyzed_chamber_data):
        summarized_total_data = self.summarize_data(analyzed_chamber_data)
        status_array = numpy.array(['Total']*len(summarized_total_data),dtype='|S25')
        summarized_chamber_data = recfunctions.append_fields(summarized_total_data,
                                                             'status',
                                                             status_array,
                                                             dtypes='|S25',
                                                             usemask=False)

        air_before_data = analyzed_chamber_data[analyzed_chamber_data['status']=='AirBefore']
        summarized_air_before_data = self.summarize_data(air_before_data)
        status_array = numpy.array(['AirBefore']*len(summarized_air_before_data),dtype='|S25')
        summarized_air_before_data = recfunctions.append_fields(summarized_air_before_data,
                                                                'status',
                                                                status_array,
                                                                dtypes='|S25',
                                                                usemask=False)
        summarized_chamber_data = recfunctions.stack_arrays((summarized_chamber_data,summarized_air_before_data),usemask=False)

        ethanol_data = analyzed_chamber_data[analyzed_chamber_data['status']=='Ethanol']
        summarized_ethanol_data = self.summarize_data(ethanol_data)
        status_array = numpy.array(['Ethanol']*len(summarized_ethanol_data),dtype='|S25')
        summarized_ethanol_data = recfunctions.append_fields(summarized_ethanol_data,
                                                             'status',
                                                             status_array,
                                                             dtypes='|S25',
                                                             usemask=False)
        summarized_chamber_data = recfunctions.stack_arrays((summarized_chamber_data,summarized_ethanol_data),usemask=False)

        air_after_data = analyzed_chamber_data[analyzed_chamber_data['status']=='AirAfter']
        summarized_air_after_data = self.summarize_data(air_after_data)
        status_array = numpy.array(['AirAfter']*len(summarized_air_after_data),dtype='|S25')
        summarized_air_after_data = recfunctions.append_fields(summarized_air_after_data,
                                                               'status',
                                                               status_array,
                                                               dtypes='|S25',
                                                               usemask=False)
        summarized_chamber_data = recfunctions.stack_arrays((summarized_chamber_data,summarized_air_after_data),usemask=False)

        return summarized_chamber_data


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("directory", help="Directory where tracking files are located")
    parser.add_argument("-o", "--overwrite", dest='overwrite', default=False, action="store_true", help="Reanalyze all experiments and overwrite previous data (default is to only analyze new data)")
    args = parser.parse_args()

    tdp = TrackingDataProcessor(args.overwrite)
    tdp.find_and_process_data(args.directory)
