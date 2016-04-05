#!/bin/bash
#*******************************************************************************
#   Ledger Blue - Misc
#   (c) 2016 Ledger
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.
#*******************************************************************************
INSTALL_DIR=$PWD/clang-arm-fropi
# Modify according to the cores you want to use 
PARALLELISM=5
git clone https://github.com/LedgerHQ/llvm
cd llvm
git checkout blue-r1
cd tools
git clone https://github.com/LedgerHQ/clang
cd clang
git checkout blue-r1
cd ../../..
mkdir llvm-build
cd llvm-build
mkdir x86
mkdir arm
cd x86
cmake -DLLVM_PREFIX="llvm-" -DLLVM_TARGETS_TO_BUILD="X86" -DLLVM_BUILD_EXAMPLES=0 -DLLVM_BUILD_TESTS=0 -DLLVM_INCLUDE_TESTS=0 -DLLVM_BUILD_DOCS=0 -DBUILD_SHARED_LIBS=0 -DCMAKE_INSTALL_PREFIX="" -DCMAKE_BUILD_TYPE=Release ../../llvm
make -j$PARALLELISM
cd ..
cd arm
CC=$PWD/../x86/bin/clang cmake -DLLVM_PREFIX="llvm-" -DLLVM_TARGETS_TO_BUILD="ARM" -DLLVM_BUILD_EXAMPLES=0 -DLLVM_BUILD_TESTS=0 -DLLVM_INCLUDE_TESTS=0 -DLLVM_BUILD_DOCS=0 -DBUILD_SHARED_LIBS=0 -DCMAKE_INSTALL_PREFIX="" -DCMAKE_BUILD_TYPE=Release ../../llvm
make -j$PARALLELISM
mkdir $INSTALL_DIR
cmake -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" -DCMAKE_INSTALL_DO_STRIP=1 -P cmake_install.cmake
rm -f $INSTALL_DIR/lib/*.a
rm -f $INSTALL_DIR/bin/llvm-*
rm -f $INSTALL_DIR/lib/clang/*/lib/linux/libclang_rt*
rm -fr $INSTALL_DIR/include
find $INSTALL_DIR/bin -not -name "clang*" -exec rm -f {} \;
rm $INSTALL_DIR/bin/clang-check $INSTALL_DIR/bin/clang-format
cd ../..
rm -rf llvm-build
rm -rf llvm

