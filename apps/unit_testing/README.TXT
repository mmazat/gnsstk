The Essential GNSS Project - Unit Testing

One of the main project goals is to provide core algorithms used in GNSS written with 'c' source code. Core alogrithms are well defined procedures with clear input and output and thus well defined test cases.

CUnit is the unit testing framework selected for this project.

CUnit
A unit testing framework for C
Anil Kumar 	<anilsaharan@users.sourceforge.net>
Jerry St.Clair 	<jds2@users.sourceforge.net>
http://cunit.sourceforge.net 


folder organization:

prj/unit_testing/
                 bin/       The executable directory and location where test files are stored.
                            i.e. run your tests here, and set your working directory (VC users) to this path.
                CUnit/      This is the CUnit framework.
                src/        This is the source directory for all test files.
                VC8/        A microsoft visual project (version 8). The CUnit library file is generated and the test program as well.
				linux/      A linux project folder to build CUnit and the test program from a simple makefile.


FAQ
1. Why 'c' source code core algorithms and unit testing? 
For maximum portability and usability, c source code is best. It is procedural and well suited for unit testing. 

2. Why only 'c' code unit testing? 
For simplicity, only the 'c' code part of the project has unit testing. The core algorithms, all in c, are best suited for simple, and generalized unit tests.

3. Are there plans for unit testing the c++ parts of the project?
No, but CPPUnit is an excellent candidate when that happens (http://cppunit.sourceforge.net/cppunit-wiki).