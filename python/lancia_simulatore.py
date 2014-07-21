#-------------------------------------------------------------------------------
# Name:        lancia_simulatore
# Purpose:
#
# Author:      tasora
#
# Created:     09/07/2014
# Copyright:   (c) tasora 2014
# Licence:     <your licence>
#-------------------------------------------------------------------------------
#!/usr/bin/env python



import subprocess
import sys
import os
from numpy import *
from matplotlib import *
from matplotlib.pyplot import *
from mpl_toolkits.mplot3d import Axes3D



# Function that can be called to execute a Chrono::Engine simuolation
# of the CES device.
# The C++ simulator is launched as an external .exe  program,
# and it will load an automatically-generated __run__.ces settings file.
# The key idea is that such __run__.ces file will be created automatically
# starting from a template.ces, where the values of some parameters, say
# voltage, rpm etc., are written accordingly to the values in the
# 'parameters' input array.
# Warning: the 'parameters' input must be a row vector of array type,
#          use the numpy library and the array object to this end.
# Warning: the 'parameters' input must contain as many values as
#          the strings in param_keys, see later in the function.

def RunChronoSimulation(parameters):

    # - 1 -
    # Configuration settings. Change to your needs!

    # Paths.
    # Note that the path of the executable, also the working directory
    # of the executable, must terminate with / slash. Also note that
    # Unix-like slashes are used in paths, instead of win backslashes \
    #
    # For example:
    #
    # directory  = "C:/tasora/code/chrono_build/bin/Release/"
    # executable = "demo_emitter.exe"
    # argument   = ""

    directory  = "C:/WeeReflex build/Corona_build/Release/"
    executable = "conveyor.exe"
    template   = "template.ces"
    argument   =  directory+"__run__.ces"
    resultfile =  directory+"results.dat"

    # Parameters.
    # These are the placeholders that must be used in the
    # 'template.ces' file, put them where you expect numbers.
    # This python procedure will replace them with the corresponding
    # parameter value from the input parameters array.
    param_keys = [  "PARAMETER_U",
                    "PARAMETER_DRUM_RPM",
                    "PARAMETER_PARTICLES_PER_SECOND" ]

    # -2-
    # Perform the automatic generation of the .ces settings file,
    # starting fromthe template and replacing key placeholders with numbers

    if (not os.path.isfile(directory+template)):
        raise Exception("The CES setting file: "+directory+template+" is not existing!")

    new_file = open(argument,'w')
    old_file = open(directory+template)
    ik = 0
    for line in old_file:
        ik = 0
        modline = line
        for key in param_keys:
            modline = modline.replace(key,  str(parameters[ik])  )
            ik = ik+1
        new_file.write(modline)
    new_file.close()
    old_file.close()


    # - 3 -
    # Launch the Chrono::Engine simulator process

    print("- Start C::E process...")

    myprocess = subprocess.Popen([directory+executable, argument],
                -1,
                None,
                None,
                subprocess.PIPE,
                subprocess.PIPE,
                None,
                subprocess._PLATFORM_DEFAULT_CLOSE_FDS,
                False,
                directory)

    # If you do not use the following, Python won't wait for the process to finish
    # and simply will go on.
    myprocess.wait()

    # Optional, see the output messages from C::E, that would have been
    # printed to the DOS console in normal execution:
    #
    #for line in myprocess.stdout:
    #    print (line)

    print("  ...end C::E process.")


    # - 4 -
    # Load result output .dat file from Chrono::Engine and
    # fill the "myresult" array with the values of .dat file

    # Initialize the result array as zeros only
    myresult=zeros(15)

    # load the output result file (assuming it is in CSV format)
    resultdata = genfromtxt(resultfile, delimiter=",")

    # process data to find useful results, store those results
    # in the 'myresult' array: (ex. do more sphisticated processing here)
    myresult = resultdata


    # - 5 -
    # Return the result array
    return myresult



#
# EXAMPLE OF USE OF THE FUNCTION
#

myparameters=array([    31000,  # voltage
                        45.8,   # drum rpm
                        5000   # particles per second
                    ])

myresults = RunChronoSimulation(c)

myparameters=array([    31000,  # voltage
                        45.8,   # drum rpm
                        300   # particles per second
                    ])

myresults = RunChronoSimulation(myparameters)

print (myresults)

