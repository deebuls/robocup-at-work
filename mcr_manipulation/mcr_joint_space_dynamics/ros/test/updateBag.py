import rosbag
import sys, getopt
import os
import glob
import json

def main(argv):
    inputfile = ''
    outputfile = ''

    try:
        opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile="])
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
    inputfile, outputfile = main(sys.argv[1:])

    with rosbag.Bag('output'+inputfile, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(inputfile).read_messages(topics=['/joint_states']):
            # This also replaces tf timestamps under the assumption 
            # that all transforms in the message share the same timestamp
            if ( msg.name[0] == "arm_joint_1" ) :
                outbag.write(topic, msg) 

    outbag.close()

