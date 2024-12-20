# Canards Processor Board Firmware

 Firmware for the 2024-2025 processor board, performing state estimation and control for the canards system.
 Project documentation can be found in the team Google Drive.

## Setup
1. Clone repo and initialize submodules `git clone --recurse-submodules git@github.com:waterloo-rocketry/cansw_processor_canards.git`
2. Open project in STM32CubeIDE (recommended IDE)
3. Use ST-Link to flash code to processor board

## Project Structure
- `src/drivers/`: custom peripheral driver modules
- `src/application/`: high-level application logic modules
- `src/third_party/`: third-party libraries
- Everything else is autogenerated by STM32CubeIDE with few modifications
