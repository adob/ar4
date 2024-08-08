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

# start the development environment
./start-env.sh
```

#### or start devbox shell
```
# start devbox shell directly if not using ./start-env.sh
bin/devbox shell
```

### install platformio
```
pip install -r requirements.txt
```

### Generate compiler_commands.json for the firmware
```
pio init --ide vscode
```

### Build and upload the firmware

```
cd firmware

pio run -t upload
# try the above command again if you receive `error writing to Teensy` the first time
```

### Run the robotool CLI
```
bt run cmd/robotool.cc
```

### Run the GUI tool
```
bt run cmd/robogui/robogui.c
```

### Run the GELLO tool
You will need the GELLO hardware for the AR4 from https://github.com/wuphilipp/gello_mechanical/tree/main/ar4.

A USB foot pedal such https://www.amazon.com/gp/product/B0B61981DP may also be handy.

```
bt run cmd/record_gello/record_gello.cc
```