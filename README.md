# TAMU RoboMaster Embedded Platform
This repository hosts the software platform that all of our robots will be running for the 2020-2021 season. How to specialize the code for each robot will be decided later.

## Design Method

Our intent is to use a state machine based design method, with a CAN polling task, UART polling task, and a task for each subsystem. Still very much a work in progress.


### Robots running this platform:
* Standard
* Hero
* Engineer
* Sentry
* Aerial

## Regenerating Code from CubeMX file

1. Clone the repository.
1. Edit CubeMX file as needed.
1. Doublecheck the "Toolchain Folder Location" (Project output location) in CubeMX's Project Manager settings is the same directory as the current CubeMX file. If it isn't, File->Save Project As to the same directory, and overwrite the file. "Toolchain Folder Location" should update itself.
1. Click "Generate Code" within CubeMX, and the existing code should be regenerated WITH user code untouched.
1. Navigate to `\Middlewares\Third_Party\FreeRTOS\Source` and replace `tasks.c` with [THIS](https://drive.google.com/file/d/1umCgfmSxtLE7y9QVOBotn8nMz5faw2_-/view?usp=sharing) updated version of the file. Size of the new file should be 172KB compared to the old 171KB.
1. Within the Keil Microvision project, under the "Project" tab on the left, right click `Middlewares/FreeRTOS` and click `Options for Group 'Middlewares/FreeRTOS'`. Navigate to the `C/C++` tab and add the flag `--c99` to the section `Misc Controls`. This is to force the compiler to compile FreeRTOS with C99, as the rest of the project is to be compiled as C++, and FreeRTOS doesn't like that.
1. Rebuild all files, and you should be ready to go.

## Features
* C++ support
* UART/CAN Transmit + Receive
* Code comments in English
* Debug by sending motor feedback to Arduino over UART ([arduino code here](https://drive.google.com/file/d/19vzUYy_eJUgesJvKe_4gClBAT4xIjfzL/view?usp=sharing)).
* That's kinda it

## Contributing
To add features (like the RC protocol pls), create a pull request and we'll go from there.
