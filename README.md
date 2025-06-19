# robotino-lowlevel-interface
This repository contains the embedded C++ module that runs onboard the Robotino 4 mobile robot. It acts as a velocity command server and manages communication with an external controller (e.g., MATLAB-based LPV-MPC).

## Functionality

- Listens for velocity commands over TCP/IP
- Applies received (vx, vy, ω) using the Robotino native API
- Reads and returns current state estimates (x, y, θ)
- Handles binary data transmission and basic error management
- Uses multi-threading for asynchronous I/O

## Requirements

- Robotino 4 running Ubuntu 20.04
- Dependencies:
  - `robotino-api2`
  - `robotino-daemon`
  - `rec-rpc`
  - `robotino-dev`
  - C++17 standard


## License

MIT License

## Contact

Elena Villalba – [elena.villalba@upc.edu](mailto:elena.villalba@upc.edu)
