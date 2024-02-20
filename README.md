# README #

This is a BRASH plugin for cFE that is based on using Juicer to generate messages needed to bridge between cFE and ROS2.

### Sections of the repository ###

* *juicer\_util* - library for interfacing with juicer generated databases
* *cfe\_msg\_converter* - a ROS2 package for generating message structures from juicer generated databases
* *cfe\_msgs* - the messages created by the cfe\_msg\_converter tool
* *cfe\_plugin* - a ROS2 package that acts as a bridge between cFE and ROS2

### Using juicer ###

* Install and build [Juicer](https://github.com/WindhoverLabs/juicer)
* Install and build [cFS](https://github.com/nasa/cFS)
    * Make sure to compile cFS with the `-g` option so that Juicer will work.
* Edit the script `juicer_util/scripts/generate_juicer_databases.py`
    * Set self.files to the list of files that should be processed.
    * Set self.fileDir to point to the location of these files.
    * Set self.outDir to the location for the generated database files.
    * Set self.exeCmd to the location of the `juicer` executable file.
    * Select if individual database files will be created or one combiled file:
        - Set `oname = self.outDir + "/" + name + ".sqlite"` for individual
        - Set `oname = self.outDir + "/" + "combined" + ".sqlite"` for combined
* Run the script to create the database files `python3 generate_juicer_databases.py`.

### Using the message converter ###

* Edit the config file `cfe_msg_converter/config/cfe_msg_converter.yaml`
    * Set `cfs_root` to point to the location of the cFS installation.
    * List the database files to process in the `juicer_db` section.
* Run the converter: `ros2 launch cfe_msg_converter cfe_msg_converter.launch.py`

### Using the cFE ROS2 bridge ###

To run:

```
ros2 launch cfe_plugin cfe_bridge.launch.py
```

This launch file accepts as argument **cfe_config** . This file contains 
configuration information for the bridge. Most of the parameters can be used
with its default values. You'll only want to modify the parameters that 
define the location of your gsw machine, specifically:

- udp_command_ip
- udp_send_port


Currently, we have available 2 configuration files to use, both
located in cfe_sbn_plugin/config:

- cfe_config.yaml: File applicable for a single-host setup. This is the default.
- cfe_config_multihost.yaml:  File applicable for a multihost setup, such as the one used by brash docker.

You can create your own file using these files as reference.

### Who do I talk to? ###

* Contact: brash@traclabs.com

