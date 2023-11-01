#!/usr/bin/env python3

import subprocess
import sys
import getopt

# This script runs juicer over a list of input files.

files_list = ['core-cpu1', 'cf/cfe_assert.so', 'cf/ci_lab.so',
                      'cf/ros_app.so', 'cf/sample_app.so', 'cf/sample_lib.so',
                      'cf/sbn_f_remap.so', 'cf/sbn.so', 'cf/sbn_udp.so',
                      'cf/sch_lab.so', 'cf/to_lab.so', 'cf/robot_sim.so',
                      'cf/cf.so']

class RunMultipleFiles():

    def __init__(self, cfe_path, juicer_path):
        self.files = files_list
        self.fileDir = cfe_path + '/build/exe/cpu1'
        self.outDir = juicer_path + '/dbs'
        self.exeCmd = juicer_path + '/build/juicer'

        for name in self.files:
            fname = self.fileDir + "/" + name
            # oname = self.outDir + "/" + name + ".sqlite"
            oname = self.outDir + "/" + "combined" + ".sqlite"
            runStr = self.exeCmd + ' --input ' + fname + \
                ' --mode SQLITE --output ' + oname + ' -v4'
            print(runStr)
            subprocess.run(runStr, shell=True)


def main(argv):

    arg_cfe_path = ""
    arg_juicer_path = ""
    arg_help = "{0} -c <cfe_path> -j <juicer_path>".format(argv[0])

    try:
        opts, args = getopt.getopt(argv[1:], "hc:j:", ["help", "cfe_path=", "juicer_path="])
    except:
        print(arg_help)
        sys.exit(2)

    for opt, arg in opts:
        if opt in ("-h", "--help"):
            print(arg_help)  # print the help message
            sys.exit(2)
        elif opt in ("-c", "--cfe_path"):
            arg_cfe_path = arg
        elif opt in ("-j", "--juicer_path"):
            arg_juicer_path = arg

    if not arg_cfe_path:
        print(arg_help)
        print("please define the path of your cFS install")
        sys.exit(2)

    if not arg_juicer_path:
        print(arg_help)
        print("please define the path of your juicer install")
        sys.exit(2)

    RunMultipleFiles(arg_cfe_path, arg_juicer_path)


if __name__ == '__main__':
    main(sys.argv)
