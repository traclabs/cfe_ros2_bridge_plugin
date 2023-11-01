#!/usr/bin/env python3

import subprocess

# This script runs juicer over a list of input files.

class RunMultipleFiles():

    def __init__(self):
        self.files = ['core-cpu1', 'cf/cfe_assert.so', 'cf/ci_lab.so',
                      'cf/ros_app.so', 'cf/sample_app.so', 'cf/sample_lib.so',
                      'cf/sbn_f_remap.so', 'cf/sbn.so', 'cf/sbn_udp.so',
                      'cf/sch_lab.so', 'cf/to_lab.so', 'cf/robot_sim.so',
                      'cf/cf.so']
        self.fileDir = '/home/swhart/code/cFS/build/exe/cpu1'
        self.outDir = '/home/swhart/code/juicer/dbs'
        self.exeCmd = '/home/swhart/code/juicer/build/juicer'

        for name in self.files:
            fname = self.fileDir + "/" + name
            # oname = self.outDir + "/" + name + ".sqlite"
            oname = self.outDir + "/" + "combined" + ".sqlite"
            runStr = self.exeCmd + ' --input ' + fname + \
                ' --mode SQLITE --output ' + oname + ' -v4'
            print(runStr)
            subprocess.run(runStr, shell=True)


def main():
    RunMultipleFiles()


if __name__ == '__main__':
    main()
