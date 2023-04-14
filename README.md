# BHS_game

This is the game backend of the BHS project.

# Before building:

* Install the latest version of [Rust](https://www.rust-lang.org/tools/install)
* go to cmakelist.txt and change `set(
  LIBTORCH_URL "https://download.pytorch.org/libtorch/cu118/libtorch-cxx11-abi-shared-with-deps-${LIBTORCH_VERSION}%2Bcu118.zip")
  with the version of your cuda
    * https://pytorch.org/get-started/locally/
* Run git submodule update --init --recursive
* if you use clion go to project folder and clion . in terminal.

# Requirements

* Install cuda and toolkit:
    * https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=20.04&target_type=deb_local
    * `sudo apt install nvidia-cuda-toolkit` DO NOT DO THIS!
* Remember after install to set the path:
    * https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#post-installation-actions

# TODOS:

1. Restructure layout to be more engine like
    - I like this layout: https://github.com/IainWinter/IwEngine/tree/master/IwEngine/src
    - But look into this one as well:
2. Make it ECS
3. Collision with arbitrary shapes: https://www.youtube.com/watch?v=MDusDn8oTSE


