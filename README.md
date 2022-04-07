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
sudo apt-get update -y && sudo apt-get upgrade -y && sudo apt-get install cmake -y
```
## Setup

1. mkdir BuzzPiPuck && cd BuzzPiPuck
2. Clone the Buzz & BuzzPiPuck Repository: git clone https://github.com/buzz-lang/Buzz.git buzz && git clone https://github.com/AidenNeale/BuzzPiPuck.git
3. Buzz Installation
4. BuzzPiPuck Installation

### Buzz Setup
```
cd buzz
mkdir build && cd build
cmake ../src/
sudo make
sudo make install
sudo ldconfig
cd ../../
```
### BuzzPiPuck Setup
```
cd BuzzPiPuck
mkdir build && cd build
cmake ../src/
sudo make
```
## Running BuzzPiPuck
