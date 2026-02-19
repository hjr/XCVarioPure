Building the XCVario Pro Software:

0) Precondition: XCVario is build using esp-idf (https://github.com/espressif/esp-idf), to be setup as described here:
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/#get-started-get-esp-idf
As OS to crossbuild XCVario software, linux e.g. latest ubuntu is strongly recommended.

1)  Clone, checkout branch: release/v5.4 install and activate esp-idf and get cmake (obviously cmake is missing from install.sh)
mkdir -p ~/esp; cd ~/esp; git clone -b release/v5.4 --recursive https://github.com/espressif/esp-idf.git;
cd ~/esp/esp-idf; 
git checkout release/v5.4;
./install.sh; . ./export.sh;
pip install cmake;
git submodule update --init --recursive;

This will get the stable branch release/v5.4 of esp-idf plus corresponding compiler, etc.
For the docker official releases, branch label is release-v5.4, see also workflow yaml.

2) Clone XCVario repository:
cd ~/esp/esp-idf/examples/get-started/; git clone <github XCVario project URL>

3) Build software and flash XCVario Software, e.g. shown as here via USB cable, or use OTA
cd XCVario
idf.py build

# If you change files in the build environment, issue the command:

idf.py reconfigure

4) In order to flash the binary through micro USB connector inside the device:
idf.py -p /dev/ttyUSB0 flash
# Then you may monitor loggings and printouts from USB serial console here:
idf.py -p /dev/ttyUSB0 monitor

# Flashing via OTA
Start OTA Software download Wifi AP at XCVario and
upload through the embedded webpage: .../xcvario_pro.bin

