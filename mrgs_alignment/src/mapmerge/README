
This is version 0.3 

REVISION HISTORY
----------------
- 0.1 - December 2007: first version (as described in
the Autonomous Robots paper)
- 0.2 - March 2008: some bug fixes
- 0.3 - September 2008: implementation of the randomized
Hough transform as described in the IROS 2008 paper. Doxygen
documentation added.

Make sure you read the associated DISCLAIMER file.

ACKNOWLEDGMENTS
---------------

The first Matlab prototype of this code (not included here)
was partially based on code developed by Andrea Censi and not available
anymore on the web  (at least I can't find it -- see references in the
papers).
The current release has been rewritten in C++ from scratch. This
release implements the improvementes based on randomization in
the Hough transform described in the IROS 2008 paper.

REFERENCES
----------
You are welcome to use this code according to the terms specified
in the DISCLAIMER file. If the results you produce are used for scholar
publications, please cite the following papers where the algorithm
was described (basic version and improved version)

- S. Carpin. "Fast and accurate map merging for multi-robot systems". 
Autonomous Robots http://dx.doi.org/10.1007/s10514-008-9097-4 
- S. Carpin. "S. Carpin. "Merging maps via Hough transform".Proceedings 
of the 2008 IEEE/RSJ International Conference on Intelligent Robots and 
Systems

Online versions of these papers are available on 
http://robotics.ucmerced.edu/Robotics/publications


COMPILING THE CODE
------------------

This code has been developed and tested on the following
systems:

- Os X 10.5.4 with gcc 4.0.1
- Ubuntu 8.04  with gcc 4.2.3 (kernel 2.6.24-16-generic)

In order to build the code you need the opencv library. It is
assumed opencv was installed in /usr/local. If a different
path was used, just edit the Makefile

To build the library, just type "make".

Please note that the Makefile included in this version
will build a shared library for Linux (i.e. a .so file).
If you want to build it for Os X comment/uncomment the appropriate
lines (see instructions in the Makefile)


DATASETS
--------
Datasets used in the papers are available in the dataset folder.
Each of them represents an occupancy grid map encoded according
to the following:
0 is an occupied cell
127 is an unknown cell
255 is a free cell

HOWTO USE THE LIBRARY
---------------------
Documentation for indidual functions and classes can be automatically generated
using Doxygen. Note that this functionality was developed and tested only with 
Doxygen 1.5.6 so it may not work with other versions. To build the documentation
just type "make doc".

See test.cpp for a simple example. In essence you need to setup two
maps that should contain cells of three types: free, occupied and unwnown.
If uncertain about the values to use, keep the defaults. After that, just
call get_hypothesis or get_hypothesis_robust. That's it. In the current 
implementation the two maps must have the same size. This is not a intrinsic
requirement of the algorithm, but rather a way to simplify the implementation.
In a future version I will (perhaps "may" is a better word) remove this requirement.

QUESTIONS OR BUGS?
------------------
Feel free to contact me.
