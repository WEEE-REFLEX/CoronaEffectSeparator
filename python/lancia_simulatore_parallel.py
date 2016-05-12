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


import threading
import time
import subprocess
import sys
import os
from numpy import *
from matplotlib import *
from matplotlib.pyplot import *
from mpl_toolkits.mplot3d import Axes3D



# Thread object that can be called to execute a Chrono::Engine simulation
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

class RunChronoSimulationThread (threading.Thread):

    # Function that initializes the parallel thread
    def __init__(self, threadID, parameters):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.parameters = parameters

    # Function that launches the chrono process
    def run(self):

        print ("Parallel thread n." + str(self.threadID) )

        # - 1 -
        # Configuration settings. Change to your needs!

        # Paths.
        # Note that the path of the executable, also the working directory
        # of the executable, must terminate with / slash. Also note that
        # Unix-like slashes are used in paths, instead of win backslashes \
        #
        # For example:
        #
        #directory  = "C:/tasora/code/chrono_build/bin/Release/"
        #executable = "demo_emit_cluster.exe"
        # argument   = ""

        directory  = "C:/Users/tasora/Desktop/build_CES/Release/"
        executable = "conveyor.exe"
        template   = "template.ces"
        argument   =  "__run__"+str(self.threadID)+".ces"

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

        new_file = open(directory+argument,'w')
        old_file = open(directory+template)
        ik = 0
        for line in old_file:
            ik = 0
            modline = line
            for key in param_keys:
                modline = modline.replace(key,  str(self.parameters[ik])  )
                ik = ik+1
            new_file.write(modline)
        new_file.close()
        old_file.close()


        # - 3 -
        # Launch the Chrono::Engine simulator process

        print("- Start C::E process...")

        myprocess = subprocess.Popen([directory+executable, directory+argument],
                    -1,
                    None,
                    None,
                    None, # subprocess.PIPE,
                    None, # subprocess.PIPE,
                    None,
                    False, # subprocess._PLATFORM_DEFAULT_CLOSE_FDS,
                    False,
                    directory)

        # If you do not use the following, Python won't wait for the process to finish
        # and simply will go on.
        myprocess.wait()

        # Optional, see the output messages from C::E, that would have been
        # printed to the DOS console in normal execution:
        #
        for line in myprocess.stdout:
            print (line)

        print("  ...end C::E process.")


        # - 4 -
        # Load result output files from Chrono::Engine
        resultdata_metal   = genfromtxt(directory+"out_distribution_metal.txt", delimiter=",")
        resultdata_plastic = genfromtxt(directory+"out_distribution_plastic.txt", delimiter=",")


        # process data to find useful results, store those results
        # ... do things here if needed


        # - 5 -
        # Store the result array in thread local data
        self.myresult = [resultdata_metal,resultdata_plastic]




# Function that can be called to execute MULTIPLE Chrono::Engine simulations
# of the CES device IN PARALLEL, to save computational time (if you assume
# they can be run in parallel, say because you have 8 cores and plenty of RAM).
# The C++ simulator is launched as an external .exe  program,
# and it will load automatically-generated __run__N.ces settings files,
# with N=0..n_threads.
# The key idea is that such __run__.ces file will be created automatically
# starting from a template.ces, where the values of some parameters, say
# voltage, rpm etc., are written accordingly to the values in the
# 'parameters' input array.
# Warning: the 'parameters' input must be a row vector of array type,
#          use the numpy library and the array object to this end.
# Warning: the 'parameters' is an array of arrays, the latter must
#          contain as many values as
#          the strings in param_keys, see later in the function.

def RunChronoSimulationInParallel(parameters):

    number_of_sets = myparameters.shape[0]

    myresults = []

    #setting: how many process do you want to run in parallel?
    nparallel = 2

    ndone = 0
    while ndone < number_of_sets:
        jparallels = min (nparallel, number_of_sets-ndone)
        ndone = ndone+jparallels
        threads = []
        # create and launch parallel threads
        for j in range(0,jparallels):
            jparameters = myparameters[j,:]
            print ("Thread " + str(j) + " parameters: ")
            print (jparameters)
            jthread = RunChronoSimulationThread(j, jparameters)
            jthread.start()
            threads.append(jthread)
        # Wait for all threads to complete
        for t in threads:
            t.join()
        for j in range(0,jparallels):
            myresults.append(threads[j].myresult)





#
# EXAMPLE OF USE
#

# define parameters for 4 simulations that will run in parallel,
# by concatenating 4 parameter arrays;

myparameters = array([
                    [  31000,  # voltage
                        45.8,   # drum rpm
                        500   # particles per second
                    ],
                    [    31000,  # voltage
                        45.8,   # drum rpm
                        1700   # particles per second
                    ],
                    [    31000,  # voltage
                        45.8,   # drum rpm
                        4000   # particles per second
                    ],
                    [    31000,  # voltage
                        45.8,   # drum rpm
                        4000   # particles per second
                    ]
                  ])


myresults = RunChronoSimulationInParallel(myparameters)

print(myresults)

print ("End main program")

