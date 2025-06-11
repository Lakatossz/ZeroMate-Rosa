# CMake generated Testfile for 
# Source directory: C:/Users/rosaj/source/repos/diplomova_prace/peripherals/reservoir
# Build directory: C:/Users/rosaj/source/repos/diplomova_prace/peripherals/reservoir/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
if(CTEST_CONFIGURATION_TYPE MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
  add_test([=[ReservoirTest]=] "C:/Users/rosaj/source/repos/diplomova_prace/peripherals/reservoir/build/Debug/reservoir_test.exe")
  set_tests_properties([=[ReservoirTest]=] PROPERTIES  _BACKTRACE_TRIPLES "C:/Users/rosaj/source/repos/diplomova_prace/peripherals/reservoir/CMakeLists.txt;57;add_test;C:/Users/rosaj/source/repos/diplomova_prace/peripherals/reservoir/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
  add_test([=[ReservoirTest]=] "C:/Users/rosaj/source/repos/diplomova_prace/peripherals/reservoir/build/Release/reservoir_test.exe")
  set_tests_properties([=[ReservoirTest]=] PROPERTIES  _BACKTRACE_TRIPLES "C:/Users/rosaj/source/repos/diplomova_prace/peripherals/reservoir/CMakeLists.txt;57;add_test;C:/Users/rosaj/source/repos/diplomova_prace/peripherals/reservoir/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
  add_test([=[ReservoirTest]=] "C:/Users/rosaj/source/repos/diplomova_prace/peripherals/reservoir/build/MinSizeRel/reservoir_test.exe")
  set_tests_properties([=[ReservoirTest]=] PROPERTIES  _BACKTRACE_TRIPLES "C:/Users/rosaj/source/repos/diplomova_prace/peripherals/reservoir/CMakeLists.txt;57;add_test;C:/Users/rosaj/source/repos/diplomova_prace/peripherals/reservoir/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
  add_test([=[ReservoirTest]=] "C:/Users/rosaj/source/repos/diplomova_prace/peripherals/reservoir/build/RelWithDebInfo/reservoir_test.exe")
  set_tests_properties([=[ReservoirTest]=] PROPERTIES  _BACKTRACE_TRIPLES "C:/Users/rosaj/source/repos/diplomova_prace/peripherals/reservoir/CMakeLists.txt;57;add_test;C:/Users/rosaj/source/repos/diplomova_prace/peripherals/reservoir/CMakeLists.txt;0;")
else()
  add_test([=[ReservoirTest]=] NOT_AVAILABLE)
endif()
subdirs("_deps/googletest-build")
