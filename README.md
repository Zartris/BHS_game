# BHS_game

This is the game backend of the BHS project.

# Before building:

* Install the latest version of [Rust](https://www.rust-lang.org/tools/install)
* go to cmakelist.txt and change `set(
  LIBTORCH_URL "https://download.pytorch.org/libtorch/cu118/libtorch-cxx11-abi-shared-with-deps-${LIBTORCH_VERSION}%2Bcu118.zip")
  with the version of your cuda
    * https://pytorch.org/get-started/locally/

# Requirements
* Install cuda and toolkit:
  * https://linuxconfig.org/how-to-install-cuda-on-ubuntu-20-04-focal-fossa-linux
* `sudo apt install nvidia-cuda-toolkit`
