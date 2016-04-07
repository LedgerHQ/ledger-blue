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


FROM        ubuntu:trusty
MAINTAINER  Ledger Firmware Team <hello@ledger.fr>

ENV PATH /opt/ledger-blue/clang-arm-fropi/bin:/opt/ledger-blue/gcc-arm-none-eabi-4_7-2013q2/bin:$PATH
ENV BOLOS_ENV /opt/ledger-blue

CMD bash

RUN apt-get update && apt-get -y install cmake git build-essential vim python wget libc6-i386 libc6-dev-i386

RUN mkdir /opt/ledger-blue

RUN cd /opt/ledger-blue && wget -O - https://launchpad.net/gcc-arm-embedded/4.7/4.7-2013-q2-update/+download/gcc-arm-none-eabi-4_7-2013q2-20130614-linux.tar.bz2 | tar xjvf -

COPY build-llvm.sh /opt/ledger-blue/build-llvm.sh

RUN cd /opt/ledger-blue && /opt/ledger-blue/build-llvm.sh

