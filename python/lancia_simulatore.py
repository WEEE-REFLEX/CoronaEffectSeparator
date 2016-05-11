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
import json




# CONFIGURATION SETTINGS.
#   Change to your needs!

# A) Paths.
# Note that the path of the executable, also the working directory
# of the executable, must terminate with / slash. Also note that
# Unix-like slashes are used in paths, instead of win backslashes \
#
# For example:
#
# directory  = "C:/tasora/code/chrono_build/bin/Release/"
# executable = "demo_emitter.exe"
# template   = "template.ces"

#directory  = "C:/Users/tasora/Desktop/build_CES/Release/"
directory  = "C:/WeeReflex build/Corona_build/Release/"
executable = "conveyor.exe"
template   = "template.ces"
argument   =  "__run__.ces"

# B) define parameters.
# These are the placeholders that must be used in the
# 'template.ces' file, put them where you expect numbers.
# This python procedure will replace them with the corresponding
# parameter value from the input parameters array.
param_keys = [  "PARAMETER_U",
                "PARAMETER_DRUM_RPM",
                "PARAMETER_MASS_PER_SECOND" ]





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

    # -1-

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
            modline = modline.replace(key,  str(parameters[ik])  )
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
    #for line in myprocess.stdout:
    #    print (line)

    print("  ...end C::E process.")


    # - 4 -
    # load the output result files (assuming in CSV format)
    resultdata_metal   = genfromtxt(directory+"out_distribution_metal.txt", delimiter=",")
    resultdata_plastic = genfromtxt(directory+"out_distribution_plastic.txt", delimiter=",")

    # process data to find useful results, store those results
    # in the 'myresult' array: (ex. do more sphisticated processing here)
    myresult = [resultdata_metal,resultdata_plastic]


    # - 5 -
    # Return the result array
    return myresult



#
# EXAMPLE OF USE OF THE FUNCTION
#
for k in range(1,6):

#myparameters=array([    31000,  # voltage
                        #45.8,   # drum rpm
                        #500   # particles per second
                    #])

#myresults = RunChronoSimulation(myparameters)

k = 0

DOE = array([[-28960, 32.67, 0.019228],
            [-31120, 35.46, 0.007928],
            [-26430, 93.73, 0.006828],
            [-25540, 75.87, 0.011753],
            [-29190, 69.92, 0.007428],
            [-29670, 122.43, 0.013278],
            [-27200, 125.22, 0.023253],
            [-25380, 78.56, 0.022228],
            [-26320, 77.7, 0.002953],
            [-28770, 119.65, 0.018053],
            [-34210, 98.34, 0.015253],
            [-32240, 66.75, 0.013703],
            [-34980, 58.4, 0.023303],
            [-31580, 64.83, 0.004978],
            [-32610, 70.69, 0.027528],
            [-27560, 82.3, 0.024378],
            [-26750, 72.61, 0.016703],
            [-30070, 104.19, 0.026753],
            [-33720, 108.42, 0.019903],
            [-29560, 89.98, 0.017403],
            [-30600, 113.6, 0.009203],
            [-32070, 64.64, 0.022303],
            [-31330, 110.72, 0.003453],
            [-33360, 95.65, 0.025253],
            [-29920, 86.72, 0.006628],
            [-30230, 122.05, 0.027028],
            [-28060, 62.43, 0.021128],
            [-33130, 127.81, 0.016053],
            [-32830, 40.16, 0.012553],
            [-34560, 116.86, 0.009503],
            [-27820, 115.14, 0.005553],
            [-30850, 89.5, 0.024253],
            [-27330, 48.32, 0.020353],
            [-34730, 56.38, 0.008328],
            [-32550, 35.94, 0.017153],
            [-26000, 91.62, 0.025428],
            [-28590, 50.53, 0.026153],
            [-25190, 106.21, 0.015453],
            [-33440, 81.25, 0.010278],
            [-30460, 52.93, 0.021553],
            [-27620, 45.25, 0.010203],
            [-31940, 43.14, 0.004528],
            [-28200, 99.68, 0.010803],
            [-33950, 59.46, 0.014753],
            [-34070, 85.09, 0.005878],
            [-26000, 54.18, 0.018553],
            [-29250, 38.43, 0.011903],
            [-25740, 102.56, 0.019528],
            [-31660, 109.57, 0.013878],
            [-26980, 45.73, 0.004028]])


for setting in DOE :

    myparameters = setting
    myresults = RunChronoSimulation(myparameters)
    k = k+1

    savetxt(directory + "sim_out/" + "inputs_" + str(k) + ".txt", myparameters)
    savetxt(directory + "sim_out/" + "distribution_metal_" + str(k) + ".txt", myresults[0])
    savetxt(directory + "sim_out/" + "distribution_plastic_" + str(k) + ".txt",  myresults[1])

		# Now, myresults[0] contains the output metal distribution
		# and myresults[1] contains the output plastic distribution
		# You can do what you like with them (ex compute recovery ratio etc.)
		#
		# ... BLA BLA
		#
		# or you can also save in a larger matrix, or in a xcel file etc.

    # Optionally show result distributions as python arrays
    # printed in console:
if False:
        print ("Output: Metal   distribution, not normalized")
        print (myresults[0])
        print ("Output: Plastic distribution, not normalized")
        print (myresults[1])


# Optionally show result normalized distributions as plot.
# Set to True or False to activate/deactivate it.
# Note that here the program HALTS until you close the plotting window!
if False:
    # 1- find the rectangle flow processor size from the __run__.ces:
    with open(directory+argument) as json_data:
        parsed_json_data = json.load(json_data)
    flowmeter_xmin =  parsed_json_data["flowmeter_xmin"]
    flowmeter_xmax =  parsed_json_data["flowmeter_xmax"]
    flowmeter_bins =  parsed_json_data["flowmeter_bins"]
    flowmeter_length = flowmeter_xmax - flowmeter_xmin
    binwidth = flowmeter_length/flowmeter_bins
    # 2- normalize curves integral
    if (myresults[0].sum()):
        my_metal = myresults[0]/ (myresults[0].sum() )
    else:
        my_metal = myresults[0] # in case no metal at all avoid division by 0
    if (myresults[1].sum()):
        my_plast = myresults[1]/ (myresults[1].sum() )
    else:
        my_plast = myresults[1] # in case no plastic at all avoid division by 0

    # plot the data
    mx = arange(flowmeter_xmin, flowmeter_xmax, binwidth)
    bar(mx, my_metal,  color="r", label="metal", alpha=0.5, width=binwidth)
    bar(mx, my_plast,  color="b", label="plastic", alpha=0.5, width=binwidth)
    xlabel('x [m]')
    ylabel('pdf')
    title('Mass distribution')
    xlim([flowmeter_xmin,flowmeter_xmax])
    grid(True)
    legend()
    show()

