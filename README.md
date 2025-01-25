## IFly, our Quadcopter Drone
Introducing IFly, our UAV (unmanned aerial vehicle) RC (remote control) quadcopter drone that can be controlled via a Logitech game controller. Along with basic flying capabilities, it consists of an ESP-32 camera that provides live video feed of the drone, which can be viewed on the user’s computer/phone. 

The drone can be used for personal enjoyment, exploration purposes or experimental projects. 

Our target audience is anyone interested in using a drone: drone enthusiasts, researchers who need an aerial device, elderly people who have limited movement, law enforcement who need to monitor dangerous areas, and delivery services. 

## How to use it?

DRONECODE - Code stored on the drone (in the Raspberry PI Pico)

OFFICIAL_ComputerArduino - Code in the Arduino UNO for receiving controller input and rf communication

OFFICIAL_XBox - Code ran on a computer with any python interpreter. Receives Xbox input and sends it to the Arduino via Serial communication

OFFICIAL_CameraWebServer - code for the ESP32-CAM module

## User Guide

### Safety: 
- Always fly in open space, away from people, wires, trees, and airports;
- Check for no fly zones in your region;
Do not fly in poor weather conditions such as raining, windy, or thunderstorms 
- Keep the drone within line of sight;
- Do not touch the propellers during operation/when spinning;
- Wear safety glasses for extra protection.

### Before Flight:
- Make sure the battery is sufficiently charged (between 14.8V to 16.8V);
- Place the drone on a flat ground, in an open space away from people;
- For safety reasons, tie the legs of the drone to some heavy objects to prevent accidents;
- Double check that the propellers are tight and there are no loose screws.

### Attaching the Battery:
- Carefully slide the battery into the space under the blue block/upper frame plate with the power cords facing the back of the drone (white arms);
- Strap the battery onto the drone with the provided velcro;
- Adjust the position of the battery so it aligns with the white line;
- Plug in the balance port to the battery checker, with the negative side plugged on the left side of the checker.


### Taking a Flight - Change Throttle:
- If the product is not pre-uploaded with the change throttle code;
- Download files in DRONECODE from https://github.com/AnnieL5/TEJ4M-Drone2024-25/tree/master 
- Open Thonny and connect to the Raspberry Pi Pico via a Micro-USB cable;
- Upload all downloaded files to Pico;
- Save the files;
- Open Arduino IDE on another computer, plug in the Arduino-RF Transmitter and open Serial Monitor;
  
    The following message should appear: 
    “Arduino TX Starting
    TX Ready…
    Please enter the message you wish to be communicated:”

- Plug in the battery to power the drone;
- The drone should beep 6 times; the gyro will then calibrate and the propellers will start rotating within 10 seconds;
- Send a number with two decimal places via the Serial Monitor to change the speed of the propellers (currently at 1.10, take-off at 1.35, landing at 1.00 and max 2.00).The motors will emergency stop once the drone has tilted more than 15 degrees in pitch or roll; power cycle to restart. 

### Taking a Flight - Controller Input:
- If the product is not pre-uploaded with the Controller Input code,

    Download files in DRONECODE from https://github.com/AnnieL5/TEJ4M-Drone2024-25/tree/ChangThrottle ;
    Open Thonny and connect to the Raspberry Pi Pico via a Micro-USB cable;
    Upload all downloaded files to Pico;
    Save the files.

- Open Arduino IDE on another computer, plug in the Arduino-RF Transmitter and the controller:

    The following message should appear: 
    “Arduino TX Starting
    TX Ready…
    Please enter the message you wish to be communicated:”
- Plug in the battery to power the drone
- The drone should beep 6 times and the gyro will calibrate;
- Press the specific button on the diagram to start the drone; it will take-off automatically
- Follow the diagram to control the drone. It will emergency stop once the drone has tilted more than 15 degrees in pitch or roll; power cycle to restart. There is also a safety time limit of 60 seconds once the drone takes off (throttle above 1.1).


### Camera - Acess the code in the CameraWebServer branch
**First time use:**

    Open Arduino IDE;
    Plug in the camera to the computer using a USB-MicroUSB cable;
    Download and open the files in the OFFICIAL_CameraWebServer from Github;
    Change the WIFI credentials to your own WIFI name and password;
    Upload the code;
    Open Serial Monitor and wait for it to provide a link to a website;
    Open the link in any browser.

**Other times:**
- Power the camera using 5V and connect to GND;
- Open WIFI/Hostpost that is recorded in the program stored inside the module;
- Wait until the camera is connected;
- Open the same website as the first time use and enjoy.


### Battery Maintenance:
- Do not puncture, wet, or throw the battery;
- Replace the battery if it puffs, has unusual smell, or smokes;
- Do not go below 3.7V per cell (can be seen from the battery checker);
- Always use a quality Lipo battery charger when charging or discharging;

**Charging**
- Never leave the battery charging unattended;
- Always balance charge the battery at a 1C rate (3.3A) to a max of 16.8V (or 4.2V per cell);
- Do not overcharge the battery.

**Discharging**
- Never discharge the battery unattended;
- Discharge battery back to storage level (3.85V per cell) if you are not going to use the drone again for a long period of time (3 days or more);
- Discharge at a rate of 1C or less (or whatever rate the charger deems appropriate);
- Never completely discharge the battery, unless it is to be disposed;
- Store the battery on a non-flammable surface and away from flammable objects; do not keep the battery plugged in if you are not using the drone.

## Thanks for checking it out!
Made by Jennifer Liu and Tianyi(Annie) Liang. 

If there are any questions or if you would like to contribute, please reach out to atianyi.liang@gmail.com. Your support is greatly appreciated! 
