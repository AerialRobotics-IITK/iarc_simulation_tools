# IARC Wave Simulator

This package contains plugins that support the simulation of waves and surface vessels in Gazebo.  

![Wave Simulation](https://github.com/srmainwaring/iarc_wave_sim/wiki/images/ocean_waves_rs750_fft.jpg)

## Notes

This is a prototype branch `feature/fft_waves` which contains an updated wave engine
that uses FFTs to generate the wavefield physics and visuals.

There are changes in the way that the wave parameters need to be set, and it may
not be possible to avoid breaking the existing interface used to specify trochoidal waves.
This is still work in progress, and the current version has a fixed set of wave parameters.

The library has additional dependencies on two FFT libraries:

- [clMathLibraries/clFFT](https://github.com/clMathLibraries/clFFT)
- [fftw](http://www.fftw.org/)

These can be installed on linux with:

```bash
sudo apt-get update && apt-get install fftw clfft
```

Check this link to install fftw in ubuntu properly -> https://stackoverflow.com/questions/45321342/how-to-build-fftw-in-ubuntu
And on macOS with:

```bash
brew fftw3 libclfft-dev libfftw3-dev
```

Aside from adding the option to use a FFT generated wavefield, the major change is
in the way that the visuals are generated. Previously the wave displacements for visuals
were generated in the shader code, the visual plugin was used to update shader parameters for wave amplitudes and frequency. Now the entire mesh for the visual is dynamically
updated in the the library then pushed into the rendering engine. This means there is no
need to maintain various sized meshes in the media files, however it does require working
around Gazebos requirement for static meshes and there is a custom Visual that implements
this. The OpenCL FFT library allows this work to be offloaded to the GPU when configured.

## Dependencies

You will need a working installation of ROS and Gazebo in order to use this package.


## Ubuntu

- Ubuntu 18.04
- ROS Melodic Morenia
- Gazebo version 9.0.0

Install CGAL 4.13 libraries:

```bash
sudo apt-get install libcgal-dev
```

### macOS

- OSX 10.11.6
- ROS Melodic Morenia
- Gazebo version 9.6.0

Install CGAL 4.13 libraries:

```bash
brew install cgal
```

## Installation

### Create and configure a workspace

Source your ROS installation:

```bash
source /opt/ros/melodic/setup.bash
source /usr/local/share/gazebo-9/setup.bash
```

Create a catkin workspace:

```bash
mkdir -p iarc_ws/src
cd iarc_ws
catkin init
```

Configure catkin:

```bash
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

### Clone and build the package

Clone the `iarc_wave_sim` repository:

```bash
cd src
git clone https://github.com/srmainwaring/iarc_wave_sim.git
```

Compile the packages:

```bash
catkin build
```

or with tests:

```bash
catkin build --catkin-make-args run_tests
```

## Usage

The wiki has details about how to configure and use the plugins:

- [WavefieldPlugin](https://github.com/srmainwaring/iarc_wave_sim/wiki/WavefieldPlugin)
- [WavefieldVisualPlugin](https://github.com/srmainwaring/iarc_wave_sim/wiki/WavefieldVisualPlugin)
- [HydrodynamicsPlugin](https://github.com/srmainwaring/iarc_wave_sim/wiki/HydrodynamicsPlugin)

## Tests

Manually run the tests:

```bash
./devel/lib/iarc_wave_sim_gazebo_plugins/UNIT_Algorithm_TEST
./devel/lib/iarc_wave_sim_gazebo_plugins/UNIT_Geometry_TEST
./devel/lib/iarc_wave_sim_gazebo_plugins/UNIT_Grid_TEST
./devel/lib/iarc_wave_sim_gazebo_plugins/UNIT_Physics_TEST
./devel/lib/iarc_wave_sim_gazebo_plugins/UNIT_Wavefield_TEST
```

## Examples

![Wave Simulation](https://github.com/srmainwaring/iarc_wave_sim/wiki/images/ocean_waves_box_example.gif)

Launch a Gazebo session with `roslaunch`:

```bash
roslaunch iarc_wave_gazebo ocean_world.launch verbose:=true
```

Publish a wave parameters message:

```bash
./devel/lib/iarc_wave_sim_gazebo_plugins/WaveMsgPublisher \
  --number 3 \
  --amplitude 1 \
  --period 7 \
  --direction 1 1 \
  --scale 2 \
  --angle 1 \
  --steepness 1
```

Publish a hydrodynamics parameters message:

```bash
./devel/lib/iarc_wave_sim_gazebo_plugins/HydrodynamicsMsgPublisher \
  --model box \
  --damping_on true \
  --viscous_drag_on true \
  --pressure_drag_on false \
  --cDampL1 10 \
  --cDampL2 1 \
  --cDampR1 10 \
  --cDampR2 1
```

For more detail see the [Example](https://github.com/srmainwaring/asv_wave_sim/wiki/Example) page in the wiki.

