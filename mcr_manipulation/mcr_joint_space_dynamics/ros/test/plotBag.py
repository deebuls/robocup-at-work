#!/home/deebuls/anaconda/bin/python
import rosbag
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from youbot_pykdl import youbot_kinematics

import sys, getopt
import os
import glob
import json

def main(argv):
    inputfile = ''
    outputfile = ''

    try:
        opts, args = getopt.getopt(argv,"hi:o:",["ifile="])
    except getopt.GetoptError:
        print 'test.py -i <inputfolder> -o <outputfile>'
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print 'test.py -i <inputfile> -o <outputfile>'
            print 'inputfolder : The folder in which all the json files are stored'
            print '              default : /tmp'
            print 'outputfile : The name of the experiments. A folder will be created '
            sys.exit()
        elif opt in ("-i", "--ifile"):
            inputfile = arg
        elif opt in ("-o", "--ofile"):
            outputfile = arg

    if (outputfile == ''):
        print 'ERROR : Please specify experiment name'
        sys.exit()
    if (inputfile == ''):
        inputfile = '/tmp'
    print 'Input file is "', inputfile
    print 'Output file is "', outputfile

    return inputfile, outputfile

if __name__ == "__main__":
    #pd.options.display.mpl_style = 'default'
    inputfile, outputfile = main(sys.argv[1:])

    kin = youbot_kinematics(open("youbot.urdf").read())

    pose = ['pose_joint_1','pose_joint_2','pose_joint_3','pose_joint_4','pose_joint_5']
    vel =  ['vel_joint_1','vel_joint_2','vel_joint_3','vel_joint_4','vel_joint_5']
    effort = ['effort_joint_1','effort_joint_2','effort_joint_3','effort_joint_4','effort_joint_5']
    end_velocity = ['vel_linear_x', 'vel_linear_y', 'vel_linear_z', 'vel_angular_x', 'vel_angular_y', 'vel_angular_z']
    columns = pose + vel + effort + end_velocity
    print columns
    df = pd.DataFrame(columns=columns)
    i = 0
    for  topic, msg, t in rosbag.Bag(inputfile).read_messages(topics=['/joint_states']):
        if ( msg.name[0] == "arm_joint_1" ) :
            joint_pose = [msg.position[0],msg.position[1],msg.position[2],msg.position[3],msg.position[4]]
            joint_vel =   [ msg.velocity[0],msg.velocity[1],msg.velocity[2],msg.velocity[3],msg.velocity[4]]
            tooltip_vel = np.dot(kin.jacobian(joint_pose), np.array(joint_vel)).tolist()[0]
            df.loc[i] = [msg.position[0],msg.position[1],msg.position[2],msg.position[3],msg.position[4],
                         msg.velocity[0],msg.velocity[1],msg.velocity[2],msg.velocity[3],msg.velocity[4],
                         msg.effort[0],msg.effort[1],msg.effort[2],msg.effort[3],msg.effort[4],
                         tooltip_vel[0], tooltip_vel[1], tooltip_vel[2], tooltip_vel[3], tooltip_vel[4], tooltip_vel[5]]

            i = i+1

    #df[pose].plot(subplots=True);
    df[vel].plot(subplots=True);
    #df[effort].plot(subplots=True);
    #df[pose].plot();
    df[vel].plot();
    df[end_velocity].plot();
    df_cumsum = df.cumsum()
    df_cumsum[vel].plot();
    df_cumsum[end_velocity].plot()
    plt.show()
    #df.to_csv(inputfile+".csv")


