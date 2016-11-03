Brickletboot for XMC1x00
========================

**This is currently in development**

Repository Content
------------------

software/:
 * examples/: Examples for all supported languages
 * build/: Makefile and compiled files
 * src/: Source code of firmware
 * generate_makefile: Shell script to generate Makefile from cmake script

datasheets/:
 * Contains datasheets for sensors and complex ICs that are used

Software
--------

To compile the C code we recommend you to install the newest CodeSourcery ARM
EABI GCC compiler
(https://sourcery.mentor.com/sgpp/lite/arm/portal/subscription?@template=lite).
You also need to install bricklib2 (https://github.com/Tinkerforge/bricklib2)
and brickletlib (https://github.com/Tinkerforge/brickletlib).
You can either clone them directly in software/src/ or clone them in a
separate folder and symlink them into software/src/
(ln -s bricklib2_path/bricklib2 project_path/software/src/). Finally make sure to
have CMake installed (http://www.cmake.org/cmake/resources/software.html).

After that you can generate a Makefile from the cmake script with the
generate_makefile shell script (in software/) and build the firmware
by invoking make in software/build/. The firmware (.bin) can then be found
in software/build/ and uploaded with brickv (click button "Flashing"
on start screen).
