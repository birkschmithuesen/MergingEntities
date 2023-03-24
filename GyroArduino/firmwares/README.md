This is a general overview and starting point for the artes mobiles controller firmware(s).
The code is based on arduino libaries and is intendet to run on an ESP32 board.
The leading codebase is "OneESP_SixMPU9250". The others may or may not be updated to have
all the features/functionality.

# Basic steps

1. Install platform.io for compiling the code

   python3 -m venv venv && ./venv/bin/pip install --upgrade pip setuptools && . ./venv/bin/activate
   pip install -U platformio

2. Change into firmware directory (where the platformio.ini file resides) and build it. The first run will install the toolchain automatically.

   pio run

3. Upload firmware to device

   pio run -t upload

   maybe specify USB port explicitly via: --upload-port=/dev/ttyUSB0

4. Use doxygen to generate in-depth firmware documentation

   doxygen DoxyFile


# References

* [platform.io](https://platformio.org/)
* [doxygen](https://www.doxygen.nl/)
