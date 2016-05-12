CoronaEffectSeparator
=====================

Simulation of a CES device, using ProjectChrono Chrono::Engine library


# Install instructions

- install Chrono::Engine, see www.projectchrono.org

- use CMake to compile this projectchrono, say you build into a directory D:\foo

- copy the directory CAD_conveyor into D:\foo\CAD_conveyor

- copy the directory objects into D:\foo\objects

- edit CAD_conveyor\settings.ces if you need

- run the exe in D:\foo\Release\

Note: if you want to modify the 3D geometry, 
rather than editing directly the .py file, it is better
that you install Chrono::SolidWorks, load the assembly of
CAD_conveyor\CAD\ into SolidWorks, modify geometries,
then use Chrono::SolidWorks to export into python .py.