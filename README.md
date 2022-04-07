# BuzzPiPuck

## Requirements
Note: This has only been tested in Linux. This 'may' work in MacOS however
there are no guarantees.

You will need Buzz, ASIO and Eigen3 Installed on your system.
 - sudo apt install libeigen3-dev libasio-dev --fix-missing -y

You will need all the dependencies Buzz Requires:
 - g++ >= 4.3 (on Linux) or clang >= 3.1 (on MacOSX)
 - cmake >= 2.8.12

In the eventuality of cmake not installing, run the following:
```
sudo apt-get update

sudo apt-get upgrade

sudo apt-get install cmake
```
## Setup

1. mkdir BuzzPiPuck && cd BuzzPiPuck
2. Clone the Buzz & BuzzPiPuck Repository: git clone https://github.com/buzz-lang/Buzz.git buzz && git clone https://github.com/AidenNeale/BuzzPiPuck.git

### Buzz Setup
1. cd buzz
2. mkdir build && cd build
3. cmake ../src/
4. sudo make
5. sudo make install
6. sudo ldconfig
7. cd ../../
### BuzzPiPuck Setup
1. cd BuzzPiPuck
2. mkdir build && cd build
3. cmake ../src/
4. sudo make

## Running BuzzPiPuck
