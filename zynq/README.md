# Cross Compilation Instructions for MacOS
Cross compiling a program on MacOS involves a bit of steps.

## Dependencies

### Packages

In order to get started, you need to install llvm and lld, which are tools used to compile your program using the embedded toolchain.

To install:

```brew install llvm ```
```brew install lld```

Installing ccmake would also be helpful but not necessary. It gives cmake a command-line gui making it easier to configure parameters.

### Repositories

To compile the embedded toolchain needed for the Zynq processor you need to clone the mechatronics embedded repository: https://github.com/jhu-cisst/mechatronics-embedded.git .

You also need to clone the mechatronics software repository: https://github.com/jhu-cisst/mechatronics-software.git . This repo is needed to link to the Amp1394 library which is a core c++ library used for the FPGA dVRK controller to read the FPGA/board state and to communicate between controllers and to the host. 

### Setup

##### Note:
Make sure you run everything in build folders to ensure the directories don't get cluttered.

#### 1. Compile Embedded Toolchain

In the [mechatronics-embedded](https://github.com/jhu-cisst/mechatronics-embedded.git) repo, run ```cmake``` on the top level directory, followed by a ```make```. This will spit out the  toolchain file named "toolchain_clang_fpgav3.cmake" along with a slew of other files 

#### 2. Cross Compile Amp1394 Libraries. 

In [mechatronics-software](https://github.com/jhu-cisst/mechatronics-software.git) repo, run 
```cmake -DCMAKE_TOOLCHAIN_FILE=pathToToolchainFile pathToSrcDir```

to configure cmake to cross compile this library using the toolcahin. This should produce a bin folder with the cross compiled Amp1394 libraries named "Amp1394Config.cmake" 

#### 3. Create CMakeLists.txt to compile your program. 

You can use the CMakeList.txt in this directory (zynq) as a template to compile and link necessary directories. 

#### 4. Cross compile your program. 

In the zynq directory (in a build folder), you need to set the toolchain file and the Amp1394 file, so you shoulr run:

```cmake -DCMAKE_TOOLCHAIN_FILE=pathToToolchainFile -DAmp1394_DIR=pathToDirOfAmp1394Config.cmake pathToSrc```

Note, the config setting to set the Amp1394 dir is pointing to the path of the directory not the path of the file. 

You can also set these parameters in a gui by running ```ccmake``` on this directory.

Then run ```make``` and this will produce an executable that is fit to run on the Zynq.