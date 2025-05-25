# NVIDIA Jetson Orin Nano Setup

INTRO: TODO. (...the Jetson is a small portable computer for robotics, etc. etc.)

It comes with a modified (and very slow) version of Ubuntu by default. That's called Jetpack, and we don't use it due to the high overhead incurred with their default setup.

We use a special image of Ubuntu Server 22.04 LTS ([download link](https://ubuntu.com/download/nvidia-jetson), [installation + setup documentation](https://pages.ubuntu.com/rs/066-EOV-335/images/Ubuntu_22.04_for_NVIDIA_Jetson_Installation_instructions.pdf)) to address this problem and have better control over the computer, though the setup process is more involved.

## First-Time Setup

### CUDA Installation

To get CUDA on Ubuntu Server, follow the steps below. **You only need to do this when setting up the Jetson from scratch** - not each time you login or something.

```console
# install the Jetson signing key
sudo apt-key adv --fetch-keys https://repo.download.nvidia.com/jetson/jetson-ota-public.asc

# add repos for Orin series (“t234”) on L4T rev 36.4
sudo add-apt-repository -y "deb https://repo.download.nvidia.com/jetson/t234 r36.4 main"
sudo add-apt-repository -y "deb https://repo.download.nvidia.com/jetson/common r36.4 main"

# add a PPA for the GPU drivers
sudo add-apt-repository -y ppa:ubuntu-tegra/updates

# grab packages from new repositories
sudo apt-get update && sudo apt-get upgrade -y

# install required packages
sudo apt-get install -y nvidia-tegra-drivers-36 libnvinfer-bin cudnn

# set user permissions for using the GPU. without them, only root may use it!
sudo usermod -a -G render,video $USER
sudo reboot
```

When it's done restarting, SSH back in and check if `nvidia-smi` shows info on CUDA/etc. :D

## Usage

### SSH

TODO: instructions on using SSH

TODO: instructions on setting up ZeroTier to communicate over the school network, and usage of the router for local networking
