# Ledger Blue development environment

Developping applications for Ledger Blue requires two compilers : 

  - A standard ARM gcc to build the non-secure (STM32) firmware and link the secure (ST31) applications
  - A customized ARM clang with [ROPI support](http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0491i/CHDCDGGG.html) to build the secure (ST31) applications    

## Getting from Docker 

A development environment image can be downloaded from [Docker Hub](https://hub.docker.com/r/nbasim/ledger-blue-sdk/)

## Building for Docker

You can build a Docker image with the provided Dockerfile - this should be the easiest way to get started if you are not using Linux

## Building on Linux 

Use the provided build-llvm.sh script file to build clang, and install a [prebuilt](https://launchpad.net/gcc-arm-embedded/4.7/4.7-2013-q2-update) gcc 

## Building on another architecture

While the build system has been untested on Mac OS X or Windows for the time being, we suggest having a look at the [install-clang](https://github.com/rsmmr/install-clang) project to build clang and the [sources or prebuilt binaries](https://launchpad.net/gcc-arm-embedded/4.7/4.7-2013-q2-update) of the ARM gcc version.  

## Contact 

For any question please contact hello@ledger.fr 


