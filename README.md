# README #

This is a BRASH plugin for cFE that is based on using Juicer to generate messages needed to bridge between cFE and ROS2.

### Sections of the repository ###

* juicer\_util - library for interfacing with juicer generated databases
* cfe\_msg\_converter - a ROS2 package for generating message structures from juicer generated databases
* cfe\_msgs - the messages created by the cfe\_msg\_converter tool
* cfe\_plugin - a ROS2 package that acts as a bridge between cFE and ROS2

### Using juicer ###

* Install and build [Juicer](https://github.com/WindhoverLabs/juicer)
* Install and build [cFS](https://github.com/nasa/cFS)
    * Make sure to compile cFS with the `-g` option so that Juicer will work.
* Edit the script `juicer\_util/scripts/generate\_juicer\_databases.py`
    * Set self.files to the list of files that should be processed.
    * Set self.fileDir to point to the location of these files.
    * Set self.outDir to the location for the generated database files.
    * Set self.exeCmd to the location of the `juicer` executable file.
    * Select if individual database files will be created or one combiled file:
        - Set `oname = self.outDir + "/" + name + ".sqlite"` for individual
        - Set `oname = self.outDir + "/" + "combined" + ".sqlite" for combined
* Run the script to create the database files `python3 generate\_juicer\_databases.py`.

### Using the message converter ###

* Edit the config file `cve\_msg\_converter/config/cfe\_msg\_converter.yaml`
    * Set cfs\_root to point to the location of the cFS installation.
    * List the database files to process in the `juicer\_db` section.
* Run the converter: `ros2 launch cfe\_msg\_converter cfe\_msg\_converter.launch.py`

### Using the cFE ROS2 bridge ###

* TODO

### Who do I talk to? ###

* Repo owner or admin
* Other community or team contact
