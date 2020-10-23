@echo off
rem connect the robot phone to laptop via USB
rem connect laptop wirelessly to the robot phone's wifi direct SSID

cd \
cd C:\Users\212027833\AppData\Local\Android\Sdk\platform-tools

rem these commands work over USB
adb disconnect
adb tcpip 5555

rem this command works over WIFI direct
adb connect 192.168.49.1:5555
