#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from pathlib import Path
import subprocess


class CreateMessageDatabase(Node):
    def __init__(self):
        super().__init__('create_message_database')

        self.get_logger().info("CreateMessageDatabase() application to parse binaries and output an SQL Database with message structure information")

        self.declare_parameter('cfs_path', "~/code/cFS")
        self.declare_parameter('juicer_path', "~/code/juicer")
        self.declare_parameter('output', "combined.sqlite")
        self.declare_parameter('files', ['core-cpu1', 'cf/cfe_assert.so', 'cf/ci_lab.so',
                               'cf/ros_app.so', 'cf/sample_app.so', 'cf/sample_lib.so',
                               'cf/sbn_f_remap.so', 'cf/sbn.so', 'cf/sbn_udp.so',
                               'cf/sch_lab.so', 'cf/to_lab.so', 'cf/robot_sim.so',
                               'cf/cf.so'])

        self.cfs_path = self.get_parameter('cfs_path').value
        self.juicer_path = self.get_parameter('juicer_path').value
        self.output = self.get_parameter('output').value
        self.files = self.get_parameter('files').value

        # cleanup args
        self.home_path = str(Path.home())
        if self.cfs_path.startswith('~') :
            self.cfs_path = self.cfs_path.replace("~", self.home_path)
        if self.juicer_path.startswith('~') :
            self.juicer_path = self.juicer_path.replace("~", self.home_path)
        if not self.output.endswith(".sqlite") :
            self.output += ".sqlite"

        self.get_logger().info("cfs_path: " + self.cfs_path)
        self.get_logger().info("juicer_path: " + self.juicer_path)
        self.get_logger().info("output: " + self.output)
        self.get_logger().info("files: " + str(self.files))

        self.cfs_binary_dir = self.cfs_path + '/build/exe/cpu1'
        self.output_dir = self.juicer_path + '/dbs'
        self.juicer_cmd = self.juicer_path + '/build/juicer'
        self.output_full = self.output_dir + "/" + self.output

        for name in self.files:
            fname = self.cfs_binary_dir + "/" + name
            oname = self.output_full
            runStr = self.juicer_cmd + ' --input ' + fname + \
                ' --mode SQLITE --output ' + oname + ' -v4'
            self.get_logger().info(runStr)
            subprocess.run(runStr, shell=True)

        self.get_logger().info("===============================================")
        self.get_logger().info("Input files: " + str(self.files))
        self.get_logger().info("Generated output file: " + self.output_full)
        self.get_logger().info("===============================================")


def main(args=None):
    rclpy.init(args=args)
    CreateMessageDatabase()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
