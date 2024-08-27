# C++ implementation of: Embedded Deformation for Shape Manipulation

## What is in this repository?
This repository contains the C++ implementation of embedded deformation (ED) [[1]](#link-to-the-original-paper). The formulation of the different cost functions used in ED are defined with the associated jacobians. In contrasts to [[1]](#link-to-the-original-paper), the optimization uses [Levenberg-Marquardt](http://ceres-solver.org/nnls_solving.html#levenberg-marquardt) from CERES instead of Gauss-Newton.

Several other cost functions are also defined, such as the minimization of model to model similarly as the one from Elastic Fusion (in the journal version).

## Dependencies

### Standard dev tools
On Ubuntu, you need to install the following packages:
```bash
sudo apt-get update
sudo apt-get install git build-essential cmake libx11-dev mesa-common-dev libgl1-mesa-dev libglu1-mesa-dev libxrandr-dev libxi-dev libxmu-dev libblas-dev libxinerama-dev libxcursor-dev libeigen3-dev libyaml-cpp-dev libceres-dev
```

## Installation instruction
To build, type into the console:
```bash
git clone git@github.com:rFalque/human_morphing.git
cd human_morphing/embedded_deformation
mkdir build
cd build
cmake ..
make -j3
```

## Run the code

> :information_source: **Info**:  The input files and the skeleton trimming method can be changed through the config.yaml file.

To run the sample, then just type:
```bash
./embedded_deformation_sample
```

To modify the inputs and outputs and the algorithm parameters, check the config file `config.yaml`

## Illustration
![example](https://github.com/rFalque/embedded_deformation/raw/master/images/screenshot.png "example of embedded deformation")

## Link to the original paper:
[1] [Embedded Deformation for Shape Manipulation](https://graphics.ethz.ch/~sumnerb/research/embdef/Sumner2007EDF.pdf)
