#--------------------------------------------------------------
#
# Example of file to be used in CMake to build an external 
# project based on Chrono::Engine
#
#--------------------------------------------------------------

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)


#--------------------------------------------------------------
#         Modify the project name if you want: 

PROJECT(CoronaElectrostaticSeparator)



#--------------------------------------------------------------
# NOTE!   use find_package() to define ChronoEngine libraries and variables, 
#         along with some optional units (components).
#         This requires that you copied FindChronoEngine.cmake in
#         the Modules/ directory of your Cmake installation. If it 
#         is not there, you can copy it in a cmake/ directory in your project, 
#         and add that directory to the search path of CMake by typing
#         set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

#find_package(ChronoEngine   COMPONENTS unit_IRRLICHT unit_POSTPROCESS unit_FEM unit_PYPARSER)
find_package(ChronoEngine   COMPONENTS unit_IRRLICHT unit_PYPARSER unit_POSTPROCESS)



#--------------------------------------------------------------
#         After the ChronoEngine package has been found, you
#         can add its include directories with the headers. 
#         Use ${CHRONOENGINE_INCLUDES} for this. 

INCLUDE_DIRECTORIES(  ${CHRONOENGINE_INCLUDES}  )


#--------------------------------------------------------------
#         Set c++ sources for building the exe, as usual in CMake

ADD_EXECUTABLE(conveyor     	source/conveyor_Ida.cpp
				source/SimulatorCES.h				
				source/UserInterfaceEventReceiver.cpp
				source/UserInterfaceEventReceiver.h
				source/ElectricParticleProperty.h )



#--------------------------------------------------------------
#         The following is needed for some platforms

SET_TARGET_PROPERTIES(conveyor PROPERTIES 
	LINK_FLAGS "${CH_LINKERFLAG_EXE} /FORCE:MULTIPLE"   )

	
#--------------------------------------------------------------
#         Uncomment the following if you want the executable to 
#         go into some specific directory: 
#            SET (CH_BINDIR_DEBUG   "" CACHE PATH "Where are your Chrono debug binaries (.dll, .exe etc.) installed?")
#            SET (CH_BINDIR_RELEASE "" CACHE PATH "Where are your Chrono release binaries (.dll, .exe etc.) installed?")
#            set_target_property(conveyor RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR_DEBUG}")
#            set_target_property(conveyor RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR_RELEASE}")
#         This done, you do not need to copy all the ChronoEngine.dll 
#         files in the output directory; also, the bin/data/ dir
#         will be shared by all programs.

# OTHERWISE, VICEVERSA, copy the .dll files in your .exe folder:
# This will help because after you compile the program, the
# ChronoEngine dll will be copied in the same directory (in case
# someone has updated ChronoEngine you do not have to remember to 
# update the copy of the dll in your folder)
# FOR WINDOWS ONLY****

SET (CH_BINDIR_DEBUG   "" CACHE PATH "Where are your Chrono debug binaries (.dll, .exe etc.) installed?")
SET (CH_BINDIR_RELEASE "" CACHE PATH "Where are your Chrono release binaries (.dll, .exe etc.) installed?")

#get_target_property( MY_RUNTIME_DIR_DEBUG  conveyor RUNTIME_OUTPUT_DIRECTORY_DEBUG)  do not work..
#get_target_property( MY_RUNTIME_DIR_RELEASE conveyor RUNTIME_OUTPUT_DIRECTORY_RELEASE)  do not work..

MESSAGE( STATUS " runtime dir debug: " ${MY_RUNTIME_DIR_DEBUG} )

ADD_CUSTOM_COMMAND(
	TARGET conveyor
	POST_BUILD
#	COMMAND ${CMAKE_COMMAND} -E copy
#		"${CH_BINDIR_DEBUG}/ChronoEngine.dll"  "${CMAKE_BINARY_DIR}/Debug/"
#	COMMAND ${CMAKE_COMMAND} -E copy
#		"${CH_BINDIR_DEBUG}/ChronoEngine_POSTPROCESS.dll"  "${CMAKE_BINARY_DIR}/Debug/"
#	COMMAND ${CMAKE_COMMAND} -E copy
#		"${CH_BINDIR_DEBUG}/ChronoEngine_PYPARSER.dll"  "${CMAKE_BINARY_DIR}/Debug/"
#	COMMAND ${CMAKE_COMMAND} -E copy
#		"${CH_BINDIR_DEBUG}/ChronoEngine_FEM.dll"  "${CMAKE_BINARY_DIR}/Debug/"
#	COMMAND ${CMAKE_COMMAND} -E copy
#		"${CH_BINDIR_DEBUG}/Irrlicht.dll"  "${CMAKE_BINARY_DIR}/Debug/"
		
	COMMAND ${CMAKE_COMMAND} -E copy
		"${CH_BINDIR_RELEASE}/ChronoEngine.dll"  "${CMAKE_BINARY_DIR}/Release/"
	COMMAND ${CMAKE_COMMAND} -E copy
		"${CH_BINDIR_RELEASE}/ChronoEngine_POSTPROCESS.dll"  "${CMAKE_BINARY_DIR}/Release/"
	COMMAND ${CMAKE_COMMAND} -E copy
		"${CH_BINDIR_RELEASE}/ChronoEngine_PYPARSER.dll"  "${CMAKE_BINARY_DIR}/Release/"
#	COMMAND ${CMAKE_COMMAND} -E copy
#		"${CH_BINDIR_RELEASE}/ChronoEngine_FEM.dll"  "${CMAKE_BINARY_DIR}/Release/"
	COMMAND ${CMAKE_COMMAND} -E copy
		"${CH_BINDIR_RELEASE}/Irrlicht.dll"  "${CMAKE_BINARY_DIR}/Release/"
		
	COMMAND ${CMAKE_COMMAND} -E copy_directory
		"${CMAKE_SOURCE_DIR}/objects"  "${CMAKE_BINARY_DIR}/objects"
)


#--------------------------------------------------------------
#  NOTE!  In the following, at least add the following line
#         in order to link all the required libraries (the main 
#         ChronoEngine library and all other libraries of units 
#         defined with the COMPONENTS parameters in find_package)
#          

TARGET_LINK_LIBRARIES(conveyor
	${CHRONOENGINE_LIBRARIES}
 	)

# this is only to simplify the GUI of CMake 
MARK_AS_ADVANCED(CMAKE_INSTALL_PREFIX)