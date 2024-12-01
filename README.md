The LucidArm is an ESP32-based prototype teleoperated robotic arm which uses LucidGloves as its mode of tracking finger position data. ESPNOW is used for wireless communication between the tracking device and robotic arm. The elbow joint is nonfunctional and still in development.
This project uses LucidGloves prototype 4.1. running on a custom version of the code.

<b>Information about and instructions for building the handtracking device can be found on the LucidVR github page:</b>
https://github.com/LucidVR/lucidgloves

<b>Official LucidVR website:</b> https://hackaday.io/project/178243-lucidgloves-vr-haptic-gloves-on-a-budget

______________________________________________________________

<b>LucidArm Hardware:</b>

STL files for 3D printing the robotic hand are located in the hardware folder. All parts are designed to be able to print on an A1 mini. It is recommended that you print the palm model and coupler mount using supports.
Joint pegs should be printed with an aligned seam so it can be easily trimmed off for smooth rotation at the joints. If there is a version of a model marked with "_1" at the end, use that instead. It is recommended that you use the .3mf file if it is provided. The materials to build the LucidGlove are not included in this list, please refer to the LucidGloves github for more infromation.

<b>For building one stationary hand, you will need:</b>
* 5x MG90S micro-servo motors
* 1x MG996R servo motor
* 1x ESP32 uPesy Wroom Devkit
* 4x M2 screws
* 5x M2.5 screws
* 1x 5V power adapter (a 12 watt iPhone charger power adapter is recommended)
* 2x 5V rechargeable batteries
* 2x micro-USB wires (.25-.5m meters or less)
  
<i>(3d printed)</i>
* 1x palm (left or right)
* 15x finger segments (each finger segment differs slightly)
* 14x standard pegs
* 12x peg caps
* 1x IMR connector peg
* 1x wristoServo peg
* 1x wrist plate
* 1x wrist cover
* 4x 60mm servo arms
* 2x Long 60mm servo arms
* <i>(optionally)</i> 1x testMountingPlate

<b>If you are building an arm including the forearm and elbow joints, you will additionally need:</b>
* 1x 5*7cm prototyping perf board
* 1x MG996 servo motor
* 8x M5 screws
  
<i>(3d printed)</i>
* 1x upper coupler
* 1x lower coupler
* 1x coupler mount
* 1x main ring
* 1x servo base
* 2x large standoffs
* 2x small standoffs
* <i>(unfinished)</i> 1x bottom plate
 
<b>If you are building the forearm battery you will need:</b>
* 4x M2 screws
* 1m of velcro strip
* 1x battery holder body
* 1x battery holder lid
* 150x80mm of soft foam (recommended to use the lid to create an outline)

______________________________________________________________
<b> LucidArm software </b>

The code for the tracking glove and the robotic hand can be found in the software folder. You first need to run the "Addresser" program on the robot arm's ESP32 to determine its MAC address before attempting to connect it to the glove; running the "Addresser" program on the
ESP32 will make it print its MAC address to Serial.

Each of the boards will continously run while powered. The robot arm should be powered before the is glove powered. Whenever the glove is connected to power, the fingers must be calibrated by forming a fist shape in order to stop the fingers on the robotic arm from jittering.
It might take several tries to get the fingers to calibrate correctly, this issue is not yet resolved.

______________________________________________________________
<b> Wiring and Power </b>

* Each of the ESP32 boards can be powered by a 5V rechargable battery with a cable connected directly to the mirco-USB port. The micro-servos should be powered by a separate 5V power adapter with a connection to the between the ground pin of the ESP32 and the ground wire of the leading to the adapter.
* If you are using the uPesy ESP32 Wroom 38pin board, it is recommended you use the pins which are already being used in the reciever code for the servo signal wires.
* Jumper wires can be used to connect the servo and board if female header pins are soldered to the board, but it is a tight fit. 




