#!/usr/bin/env python

import rosbag
import json
import re
import yaml

import os
from duckietown import DTROS
from rospy_message_converter import  message_converter
from decimal import Decimal
from decimal import *

class Ros_Analyze(DTROS):

    def __init__(self, node_name):
        super(Ros_Analyze, self).__init__(node_name=node_name)

    @staticmethod
    def stamp2time(stamp):
        return stamp.get('secs') + stamp.get('nsecs')/1000000000.0

    @staticmethod
    def stamp2time_decimal(stamp):
        a = float(stamp.get('secs'))
        b = stamp.get('nsecs')/1000000000.0

        c = (Decimal(a+b).quantize(Decimal('.000000001'), rounding=ROUND_DOWN))

        return c

    @staticmethod
    def retrieve_latencies_lanepose_settings(bag):
        # This function retrieves the latency from and up to the detector node, the lane_pose estimation of the duckiebot
        # as well as the global variables that have been set
        lat = {'time': [], 'meas': []}
        set = {'gain': [], 'trim': [], 'baseline': [], 'radius': [], 'k': [], 'limit': [], 'omega_max': [], 'v_max': []}
        lane_pose = {'header': {'stamp': {'secs': [], 'nsecs': []}},'time': [],'time_rel': [], 'd': [], 'phi': [], 'd_ref': [],  'phi_ref': []}
        find_msg_re = r'^(\[LineDetectorNode\] \d+:\sLatencies:\s)'
        find_line_re = r'\s+--pub_lines--\s+\|\s+total\s+latency\s+\d+.\d+ ms\s+'
        find_line_set = r'gain:\s'
        find_float = r'\d+\.\d+'
        start = True

        # for _, msg, _ in bag.read_messages(topics=['/diagnostics']):
        #     temp = message_converter.convert_ros_message_to_dictionary(msg)
        #     print(temp)

        #in case the Duckiebot is running on daffy
        for _, msg, _ in bag.read_messages(topics=['/autobot01/lane_filter_node/lane_pose']):
            temp = message_converter.convert_ros_message_to_dictionary(msg)

            if start:
                start_time = Ros_Analyze.stamp2time_decimal(temp.get('header').get('stamp'))
                start = False
                lane_pose['header']['stamp']['secs'] = temp.get('header').get('stamp').get('secs')
                lane_pose['header']['stamp']['nsecs'] = temp.get('header').get('stamp').get('nsecs')

            time = Ros_Analyze.stamp2time(temp.get('header').get('stamp'))

            lane_pose['d'].append(temp['d'])
            lane_pose['phi'].append(temp['phi'])
            lane_pose['phi_ref'].append(temp['phi_ref'])
            lane_pose['d_ref'].append(temp['d_ref'])
            lane_pose['time'].append((time))
            time = Ros_Analyze.stamp2time_decimal(temp.get('header').get('stamp'))
            lane_pose['time_rel'].append(float(time-start_time))

        #in case the Duckiebot is running on Master19
        for _, msg, _ in bag.read_messages(topics=['/autobot01/lane_controller_node/lane_pose']):
            temp = message_converter.convert_ros_message_to_dictionary(msg)

            if start:
                start_time = Ros_Analyze.stamp2time_decimal(temp.get('header').get('stamp'))
                start = False
                lane_pose['header']['stamp']['secs'] = temp.get('header').get('stamp').get('secs')
                lane_pose['header']['stamp']['nsecs'] = temp.get('header').get('stamp').get('nsecs')

            time = Ros_Analyze.stamp2time(temp.get('header').get('stamp'))

            lane_pose['d'].append(temp['d'])
            lane_pose['phi'].append(temp['phi'])
            lane_pose['phi_ref'].append(temp['phi_ref'])
            lane_pose['d_ref'].append(temp['d_ref'])
            lane_pose['time'].append((time))
            time = Ros_Analyze.stamp2time_decimal(temp.get('header').get('stamp'))
            lane_pose['time_rel'].append(float(time-start_time))

        for _, msg, _ in bag.read_messages(topics=['/rosout']):
            temp = message_converter.convert_ros_message_to_dictionary(msg)
            msg_string = temp.get('msg')
            #print(temp.get('name'))
            if temp.get('name') == '/{}/line_detector_node'.format(os.environ['DUCKIEBOT']) and re.search(find_msg_re, temp.get('msg')):
                time = Ros_Analyze.stamp2time(temp.get('header').get('stamp'))
                for line in msg_string.split('\n'):
                    if re.search(find_line_re,  line):
                        snippet = re.findall(find_line_re,  line)[0]
                        lat['time'].append(time)
                        lat['meas'].append(re.findall(r'\d+.\d+',snippet)[0])

            if temp.get('name') == '/{}/kinematics_node'.format(os.environ['DUCKIEBOT']):
                for line in msg_string.split('\n'):
                    if re.search(find_line_set,  line):
                        snippet = re.findall(find_float,  line)
                        set['gain'].append(snippet[0])
                        set['trim'].append(snippet[1])
                        set['baseline'].append(snippet[2])
                        set['radius'].append(snippet[3])
                        set['k'].append(snippet[4])
                        set['limit'].append(snippet[5])
                        set['omega_max'].append(snippet[6])
                        set['v_max'].append(snippet[7])

        return lat, set, lane_pose


    @staticmethod
    def retrieve_segment_count(bag):
        # This function retrieves the number of segments detected at each time by the line_detector_node
        segs = {'time': [], 'meas': []}

        for _, msg, _ in bag.read_messages(topics=['/{}/line_detector_node/segment_list'.format(os.environ['DUCKIEBOT'])]):
            temp = message_converter.convert_ros_message_to_dictionary(msg)
            time = Ros_Analyze.stamp2time(temp.get('header').get('stamp'))

            segs['time'].append(time)
            segs['meas'].append(len(temp.get('segments')))
        return segs

    @staticmethod
    def retrieve_update_freq(bag):
        # This function retrieves all the update frequencies, number of connections and message counted for each node
        # that was subscribed to
        freq = {'node': [], 'frequency': [], 'message_count': [], 'connections': []}
        dict = bag.get_type_and_topic_info().topics

        for node in dict:
            freq['node'].append(node)
            freq['frequency'].append((dict[node])[3])
            freq['message_count'].append((dict[node])[1])
            freq['connections'].append((dict[node])[2])

        return freq


    def run(self):

        with rosbag.Bag('/data/'+os.environ['BAGNAME']+'.bag', 'r') as bag:
            segs = self.retrieve_segment_count(bag)
            lat, set, lane_pose = self.retrieve_latencies_lanepose_settings(bag)
            freq = self.retrieve_update_freq(bag)

        # save the node information (update freequency etc) into a .json file
        with open('/data/{}_node_info.json'.format(os.environ['BAGNAME']), 'w') as file:
            #print(lat)
            file.write(json.dumps(freq))

        # save the latency information into a .json file
        with open('/data/{}_latencies.json'.format(os.environ['BAGNAME']), 'w') as file:
            #print(lat)
            file.write(json.dumps(lat))

        # save the lane pose estimation of the Duckiebot into a .json file
        with open('/data/{}_lane_pose.json'.format(os.environ['BAGNAME']), 'w') as file:
            #print(lat)
            file.write(json.dumps(lane_pose))

        # save the constants information into a .json file
        with open('/data/{}_constant.json'.format(os.environ['BAGNAME']), 'w') as file:
            #print(lat)
            file.write(json.dumps(set))

        # save the segment count information into a .json file
        with open('/data/{}_segment_counts.json'.format(os.environ['BAGNAME']), 'w') as file:
            #print(segs)
            file.write(json.dumps(segs))

if __name__ == '__main__':
    node = Ros_Analyze(node_name='ros_Analyze')
    node.run()
