Next Steps
	- Currently, the wrist rotation and grabber don't work
	- The inverse kinematics need to be rebuilt from scratch
	- Possible integration with groov rio for receiving instructions
	- Possible integration with CV cameras to locate boxes and
	  box orientation to guide the robot arm movement

Important Code
	- main.c is legacy code that contains useful definitions 
	  and other information for future use and reference
	- example.c is run by main.c, it also contains useful 
	  code for reference and use
	- testing_main.c is the newest version of the code, containing
	  the bare minimum to move the limbs of the robot, and is built
	  from code in main.c and example.c. I would recommend building 
	  this as the new main file, while keeping main.c and example.c,
	  for reference. 

Gitlab Information
	- cooper_testing_F2021 is the most up to date branch

Computer Architecture
	- One STM32 F401RE is the microcontroller
	- Three xnucleoihm02a1 stacked in daisy chain configuration
	  each controls two microstepping motors

Running Code
	- Make sure that the microcontroller is connected to power
	- Use the Keil uVision5 IDE (I already had this from MTE 241)
	- Make sure to have the ST-link_utility setup complete
	- Plug the black USB coming from the STM32 f401RE into your labtop
	- In Configure Flash Tools (in the keil IDE), go to C/C++, and set 
	  optimization to level 2
	- Rebuild all target files
	- Flash the code to the microcontroller
	- Either run the code through debug mode, or hit the reset button 
	  (the black button) on the STM32 f401RE

ST-link_utility
	- Run the setup in here to allow for USB connection to the
	  microcontroller from your laptop

X-Cube-SPN2.chm
	- Documentation for the X-CUBE-SPN2 motor driver software
	- This software provides functions for the xnucleoihm02a1 to 
          control the microstepping motors that control the arm
        - If the file is blank, right click on it, go to properties, 
          and check "unblock"

X-Cube-SPN2.pdf
	- An official explanation of what X-Cube-SPN2 is

inverse_kinematics_resources.docx
	- Aggregation of resources to help rebuild the inverse
	  kinematics from scratch (physics for control of arm)

HAL_documentation.pdf
	- Documentation for software that allows for interaction 
	  with the hardware
	- Hardware Abstraction Layer

X-NUCLEO-IHM02A1_documentation.pdf
	- Documentation for stepper motor controller board

system_architecture.docx
	- Contains a diagram of the robot
	- Contains an electrical wiring diagram
	- Contains solutions to common electrical/hardware issues