### Building the firmware

## clone the repo

```
# install git if not already installed
sudo apt install git
git clone --recurse-submodules https://github.com/adob/ar4.git
cd ar4
```

### Install Nix

Install Nix following the instructions at https://nixos.org/download/

### sandboxing (optional)
This protects your files from being directly accessed by the code in this repository and its dependencies
```
# install firejail if not already installed
sudo apt install firejail
./start-env.sh
```

#### or start devbox shell
```
bin/devbox shell
```
### Build the firmware

```
cd firmware

pio run -t upload
# try the above command again if you receive `error writing to Teensy` the first time
```
