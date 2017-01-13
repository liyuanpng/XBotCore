# XBotCore

XBotCore is the new software architecture to control ADVR robots: it uses Xenomai API to satisfy Real-Time requirements. 

Moreover it provides XDDP pipes communication with a Not-Real-Time communication API. 

Here you can find the [documentation](http://95.110.214.65/XBotCore/index.html).

## How to use WALKMAN robot with XBotCore and YARP

## *Current network topology*

### **WALKMAN RT PC**

*hostname*:         walkman-exp

*username*:         embedded

*password*:         send an email to luca.muratore@iit.it or alessio.rocchi@iit.it for the password

*operating system*: Debian stretch

*local IP address*: 10.24.3.100

*description*:    
The WALKMAN RT PC is responsible for the RT communication with the robot (EtherCAT based).  
It has two ethernet connections:  
* connection with the robot EtherCAT slaves network
* connection with external control pc (control modules)

*what to run*:      

* XBotCore
* yarpserver
* XBotYARP

*how to run them*:

Check the connection between the robot and the WALKMAN RT PC, power on the robot and have always the emergency stop in your hand.  
Start RTNET:  
~$ **ec_rtnet_start**  

Open 3 terminal on WALKMAN RT PC:

* On terminal # 1 start XBotCore: it will start the motors using the control mode and the gains specified in the YAML config file passed as an argument.  
~$ **cd $XBOTCORE_ROOT**  
~$ **XBotCore configs/config_walkman.yaml**  

* On terminal # 2 start YARP name server: check that it is bind the the local address (10.24.3.102)   
~$ **yarpserver --write**  

* On terminal # 3 start XBotYARP: it will start the Not-Real-Time YARP communication with XBotCore opening all the YARP ports needed to communicate with the robot using YARP (like in gazebo with the robot model).  
~$ **cd $XBOTCORE_ROOT**  
~$ **XBotYARP configs/config_walkman.yaml**

### **WALKMAN external control PC**

*hostname*:         walkman-pilot-pc

*username*:         lucamuratore

*password*:         send an email to luca.muratore@iit.it or alessio.rocchi@iit.it for the password

*operating system*: Ubuntu 14.04

*local IP address*: 10.24.3.77

*description*:    
The WALKMAN external control PC is responsible for the YARP control module.  
It has two ethernet connections:  
* internet connection
* connection with the RT PC

*what to run*:      

* your control module

*how to run them*:

Check the connection between the external pc and the WALKMAN RT PC, check that $ROBOTOLOGY_PROFILE is equal to ROBOT, otherwise execute:  
~$ **cd $XBOTCORE_ROOT/profiles/robot && ./activate.sh**  
Refresh all the terminal. 
Configure yarpserver (that is running on RT PC):  
~$ **yarp conf 10.24.3.102 10000**   
Check with:  
~$ **yarp detect** 

You are now ready to run your module.  
If you want to go back in simulation execute:  
~$ **cd $XBOTCORE_ROOT/profiles/simulation && ./activate-local.sh**  
Refresh all the terminal.