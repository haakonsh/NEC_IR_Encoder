
NEC IR Encoder
######

Overview
********
This application will output NEC IR Encoded frames on p0.11 using the nRF52's PWM0 peripheral.

Requirements
************
nRF52 series device with at least one available PWM. NCS v2.2.0 SDK or newer. 

Building and Running
********************
Using the nRF Connect for VS Code extension:

Click the '+' button labeled "nRF Connect: Add Folder As Application" in the APPLICATIONS pane.
You will need to hover over the pane beyfore the button row will appear.

Add a build configuration for your nRF52 device, then build and flash the device.

Additional info
***************
[NEC IR protocol appnote from Altium](https://techdocs.altium.com/display/FPGA/NEC%2bInfrared%2bTransmission%2bProtocol)

I recommend the NEC Decoder extension for Saleae Logic 2 if you need to decode NEC IR packets.
