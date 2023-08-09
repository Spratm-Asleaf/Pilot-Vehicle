> Online supplementary materials of the paper titled 
> 
> "A Railway Accident Prevention System Using An Intelligent Pilot Vehicle"
> 
> Authored By: Shixiong Wang, Xinke Li, Zhirui Chen, and Yang Liu
> 
> From the Department of Industrial Systems Engineering and Management and the Department of Civil and Environmental Engineering, National University of Singapore
> 
> @Author: Shixiong Wang
> 
> @Date: 08 Aug 2023
> 
> @Site: https://github.com/Spratm-Asleaf/Pilot-Vehicle

$~$

PSOPT: https://github.com/PSOPT/psopt 

This file instructs how to install PSOPT on Ubuntu.

This file is adapted from PSOPT's official instructions at: 

https://github.com/PSOPT/psopt/blob/master/README_Ubuntu_22.04.md 

which serves as a helper.

## PSOPT installation on Ubuntu 20.04
If you use Ubuntu **20.04**, all dependencies plus GNUplot can simply be installed as follows:

````
sudo apt-get install git
sudo apt-get install cmake
sudo apt-get install gfortran
sudo apt-get install g++
sudo apt-get install libboost-dev
sudo apt-get install libboost-system-dev
sudo apt-get install coinor-libipopt-dev
sudo apt-get install libcolpack-dev
sudo apt-get install libadolc-dev
sudo apt-get install gnuplot
sudo apt-get install libeigen3-dev
sudo apt-get install libblas-dev
sudo apt-get install liblapack-dev
````

## PSOPT installation on Ubuntu 22.04

With Ubuntu **22.04**, a runtime error related to the ADOL-C library is currently reported when executing PSOPT code if you follow the instructions that work for Ubuntu 20.04. 
Here are the specific instructions that are needed to install PSOPT under Ubuntu 22.04.

First, you should run the following from a terminal window to install some packages that are required:

````
sudo apt-get install git
sudo apt-get install cmake
sudo apt-get install gfortran
sudo apt-get install g++
sudo apt-get install libboost-dev
sudo apt-get install libboost-system-dev
sudo apt-get install coinor-libipopt-dev
sudo apt-get install gnuplot
sudo apt-get install libeigen3-dev
sudo apt-get install libblas-dev
sudo apt-get install liblapack-dev
````

Second, you should run the following commands to download, compile, and install the ADOLC and ColPack.

## Manually install "colpack" and "adolc"

Download the ADOLC Package

``
wget --continue www.coin-or.org/download/source/ADOL-C/ADOL-C-2.6.3.tgz
``

Extract

``
tar zxvf ./ADOL-C-2.6.3.tgz
``

Download the ColPack Package

``
wget --continue http://archive.ubuntu.com/ubuntu/pool/universe/c/colpack/colpack_1.0.10.orig.tar.gz
``

Extract

``
tar zxvf ./colpack_1.0.10.orig.tar.gz
``

You can rename the folder

``
mv ColPack-1.0.10 ColPack
``

Complie and configure ColPack

````
cd ColPack
sudo ./autoconf.sh
sudo make
sudo make install
````

### If you succeed, you should see some "lib" files in the folder "build", then move them to "/usr/lib"; if you cannot see, you failed to compile ColPack for some reasons

``
sudo cp -P ./build/lib/libCol* /usr/lib
``

### Compile and configure ADOL-C with ColPack; you MUST use the absolute path of the folder "ColPack/build", supposing it is "AbsolutePath"; remember to replace it with your real absolute path

````
sudo ./configure --enable-sparse --with-colpack=AbsolutePath/ColPack/build
sudo make
sudo make install
````

### If you succeed, you should see the "adolc_base" folder somewhere in your computer; it was created by the compilation process. In my case, it was placed in the "root" directory (i.e., "/root/adolc_base"). Copy the files below to "/usr/lib"

````
sudo cp -P /root/adolc_base/lib64/lib* /usr/lib
sudo cp -r /root/adolc_base/include/* /usr/include/
````

NOTE: If you cannot see the "adolc_base" folder, you failed to complie the ADOLC for some reasons

NOTE: If you cannot copy the files due to permission issues, try to enter "/root/" first and use chmod command to obtain permission of the folder "adolc_base". Then try the two statements above.

Move or copy the following two files to the folder "/usr/lib/pkgconfig"
````
adolc.pc
ColPack.pc
````

Then, you can run the following commands to download, compile and install PSOPT.

````
git clone https://github.com/PSOPT/psopt.git
cd psopt
mkdir build
cd build
sudo cmake -DBUILD_EXAMPLES=ON ..
sudo make
sudo make install
````
