# Canards Processor Board Firmware
Firmware for the 2024-2025 processor board, performing state estimation and control for the canards system. Project documentation can be found in the team Google Drive.

## Project Structure
- `src/drivers/`: custom peripheral driver modules
- `src/application/`: high-level application logic modules
- `src/third_party/`: third-party libraries
- `src/common/`: shared resources specific to this project
- `tests/`: everything for [testing](#Unit-Testing)
- Everything else is autogenerated by STM32CubeIDE with few modifications

## Developer Setup
This project is not dependent on STM32CubeIDE.
Code editing, unit testing, and building should be done in the devcontainer.
Only running/debugging on target should be done in STM32CubeIDE.

#### 1. Clone repo
- Clone repo and initialize submodules: ```
   git clone --recurse-submodules https://github.com/waterloo-rocketry/cansw_processor_canards ```

#### 2. Edit, test, build in devcontainer
The devcontainer contains the necessary environment setup for editing, unit testing, and building. Most software development work should occur here.
- Open the project using vscode devcontainers.
  - [Install devcontainers](https://code.visualstudio.com/docs/devcontainers/tutorial)
  - In a new vscode window, use `Ctrl+Shift+P`, search `Dev Containers: Open Folder In Container...`, then select this project folder
- Use `/scripts/run.sh` from a terminal inside the devcontainer to build the project and/or run unit tests:

  ```bash
  ./scripts/run.sh help
  ```

#### 3. Run/debug in STM32CubeIDE
STM32CubeIDE is preferred for flashing/debugging on hardware. NOTE: STM32CubeIDE is not set up to build this project. STM32CubeIDE is only used to run the build from the previous step.
- Import the project into STM32CubeIDE (version 1.16.1 recommended): `File->Import...->Existing Projects into Workspace`
- Build the project (previous step)
- Use an ST-Link programmer to connect to processor board.
- Use STM32CubeIDE run/debug as usual. Launch configurations are persisted in the `.launch` files in this repo.
  - NOTE: since the project can't be built in STM32CubeIDE, auto-building before launch is turned off. You must manually build the project before launching if there are any changes.

#### *ALTERNATIVE 3. Run/debug via Vscode STM32 extension*
*Instead of using STM32CubeIDE, run/debug in vscode is possible using the STM32 vscode extension ([setup info here](https://community.st.com/t5/stm32-mcus/how-to-use-vs-code-with-stm32-microcontrollers/ta-p/742589) ). This was omitted from the devcontainer because usb passthrough into devcontainers is untrivial.*

## Testing
The project uses GoogleTest and Fake Function Framework (fff) for unit testing. All testing-related files are in `tests/`.
- Tests are built from `tests/CMakeLists.txt` which is separate from the project's main build config. Building and running tests is done from `scripts/run.sh` described above.
- Test source code should be written in `tests/unit/`.
- Mocks should be made with fff in `tests/mocks/`

## Code Standards
- This project follows the [team-wide embedded coding standard](https://docs.waterloorocketry.com/general/standards/embedded-coding-standard.html).
- Use clang-format for automatic code formatting. The script must be run from the project root directory:
  ```bash
  ./scripts/format.sh
  ```

- Rocketlib is included at `src/third_party/rocketlib`.
- Developers should be aware of the BARR Standard and make an effort to follow it.
