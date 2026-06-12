# Portable Ground Station

This directory contains the V1.2 revision of the ground station. It no longer has iss several known problems;

### Updates
- Removed I2C ADC, switched to ESP32-ADCs
- Using direct USB+/- for programming, removed uart bridge
- Added V_BCKP for GPS, (copied circuit from gps antenna board)

### Future Suggestions
- Look into using the MAX17043 for battery management
- Look into using TP4056 for battery charging (cheaper)
- switch to a new battery switch cause it breaks