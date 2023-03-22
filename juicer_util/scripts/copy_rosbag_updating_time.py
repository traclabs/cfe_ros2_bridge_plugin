"""Copy rosbag changing timestamp."""

from __future__ import annotations

from typing import TYPE_CHECKING, cast
from pathlib import Path
import glob

from rosbags.interfaces import ConnectionExtRosbag2
from rosbags.rosbag2 import Reader, Writer
from rosbags.serde import deserialize_cdr, serialize_cdr
from rosbags.typesys import get_types_from_msg, register_types



class CopyRosbagUpdatingTime():

    def __init__(self):
        print("Initialized.")

    def offset_timestamps(self, src: Path, dst: Path, timestamp_to_use: str) -> None:
        """Offset timestamps.

        Args:
            src: Source path.
            dst: Destination path.
            timestamp_to_use: Either ROS2 or CCSDS

        """
        with Reader(src) as reader, Writer(dst) as writer:
            conn_map = {}
            for conn in reader.connections:
                ext = cast(ConnectionExtRosbag2, conn.ext)
                conn_map[conn.id] = writer.add_connection(
                    conn.topic,
                    conn.msgtype,
                    ext.serialization_format,
                    ext.offered_qos_profiles,
                )

            for conn, timestamp, data in reader.messages():
                # initialize timestamp to current one
                new_timestamp = timestamp
                msg = deserialize_cdr(data, conn.msgtype)
                if timestamp_to_use == 'ROS2':
                    if head := getattr(msg, 'header', None):
                        new_timestamp = head.stamp.sec * 10**9 + head.stamp.nanosec
                        print("Timestamp from " + timestamp_to_use + " header = " + str(new_timestamp))
                else:  # default to CCSDS if not ROS2
                    if head := getattr(msg, 'telemetry_header', None):
                        if sec := getattr(head, 'sec', None):
                            time = getattr(sec, 'time', None)
                            if time.any():
                                new_timestamp = self.get_timestamp(time)
                                print("Timestamp from " + timestamp_to_use + " header = " + str(new_timestamp))
                            else:
                                print("Didn't find time attribute.")
                        else:
                            print("Didn't find sec attribute.")
                    else:
                        print("Didn't find telemetry_header attribute.")

                writer.write(conn_map[conn.id], new_timestamp, data)

    def get_timestamp(self, time):
        retval = 0
        # From CFE code - Time, big endian: 4 byte seconds, 2 byte subseconds
        secs = [time[0], time[1], time[2], time[3]]
        subsecs = [time[4], time[5]]
        bytesecs = bytes(secs)
        bytesubsecs = bytes(subsecs)
        intsecs = int.from_bytes(bytesecs, byteorder='big', signed=False)
        intsubsecs = int.from_bytes(bytesubsecs, byteorder='big', signed=False)
        epoch_delta = 315532800
        newsecs = intsecs + epoch_delta
        conversion_factor_subsec_to_nsec = 1000000000.0 / 65536.0
        nsecs = int(float(intsubsecs) * conversion_factor_subsec_to_nsec)
        retval = newsecs * 1000000000 + nsecs
        return retval

    def register_msgs(self, path_list):
        add_types = {}
        for pathstr in path_list:
            msgpath = Path(pathstr)
            msgdef = msgpath.read_text(encoding='utf-8')
            add_types.update(get_types_from_msg(msgdef, self.guess_msgtype(msgpath)))

        register_types(add_types)

    def guess_msgtype(self, path: Path) -> str:
        name = path.relative_to(path.parents[2]).with_suffix('')
        if 'msg' not in name.parts:
            name = name.parent / 'msg' / name.name
        return str(name)


def main():
    time_to_use = 'ROS2'
    msg_dir = '/home/tmilam/code/galactic_ws/brash/src/cfe_ros2_bridge_plugin/cfe_msgs/msg'
    src_path = "/home/tmilam/code/galactic_ws/brash/new-bag-file"
    dst_path = "/home/tmilam/code/galactic_ws/brash/new-bag-file-" + time_to_use
    print("Updating to " + time_to_use + " times.")
    updated_rosbag = CopyRosbagUpdatingTime()
    msg_files = glob.glob(msg_dir + '/*.msg')
    updated_rosbag.register_msgs(msg_files)
    updated_rosbag.offset_timestamps(Path(src_path), Path(dst_path), time_to_use)

if __name__ == '__main__':
    main()

