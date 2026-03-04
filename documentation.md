# idk robot

### Section 1: Networking
OpenMesh OM5P-ac radios flashed with OpenWRT are used for robot-to-robot communication. They are flashed using (this)[https://github.com/ap51-flash/ap51-flash] tool, and then placed in relay mode using (this)[https://openwrt.org/docs/guide-user/network/wifi/relay_configuration] guide. Keep in mind one port must be a WAN port to download the necessary extensions. Ensure the master radio is on 10.XX.XX.1, and the other radios have an IP on that subnet. Flashing the radios is time-intensive and finnicky, and the hardware itself is very outdated. 

Once the radios are set up, they must be given either static IPs or static DHCP leases. This can be done in the LuCi interface at Networking > DHCP > Static Leases. A third radio can also be used for any other laptops. The LuCi credentials are [admin, nerdspark]@10.93.12.1 and the WPA is [SparkNet, nerdspark]. 

The network is laid out as follows:
- 10.93.12.1: master radio
- 10.93.12.2: master PC
- 10.93.12.3: slave radio
- 10.93.12.5: slave PC (not DHCP)
- 10.93.12.10: master OPI 1 (front)
- 10.93.12.11: slave OPI 1 (front)
- 10.93.12.12: master OPI 2 (back+top)
- 10.93.12.13: slave OPI 2 (back+top)
In general, even IPs are on the master and odd IPs are on the slave.

In terms of NetworkTables, a server is run on the master computer and clients are run on the slave computer and OPIs. Enable/disable and joystick values are sent through NT, along with various mission-critical flags and poses (on top of logging). Networktables has various poorly-documentd quirks, including update frequencies and custom datatypes, which are frequently implemented throughout the codebase. The NT epoch is based on the startup of the master computer, so all timestamps on the slave and master are as well. Ideally, each robot should run its own server to minimize latency, and NT values should be funneled from the slave to the master.

Errata:
- if NT devices on the slave start disconnecting, re-deploy/restart code
- radio boot-up is even longer in this custom firmware than it is on FRC bots.

### Section 2: Hardware
Herk Pulsar mini PCs are used as the primary hardware. They run Ubuntu 21.10 LTS, which is required for the CTRE libraries. Visual installation was used in order to set up VSCode and WPILib extensions; however, it is unclear if either of those are necessary. The strictly necessary packages are listed below. Certain ports must also be opened in the firewall, for DS communication, Phoenix Tuner, ssh, and NetworkTables. The canivore must be connected via USB to the mini PC, and the caniv CLI utility can be used to manage connections.
- openjdk-17 (or later)
- gradle
- [canivore-usb](https://v6.docs.ctr-electronics.com/en/stable/docs/installation/installation-nonfrc.html)
- open-ssh
To actually run code on the robot, you must run the deploy script. It uses rsync to copy over the necessary files, and then runs the program through ssh, using either simulate.sh or `./gradlew simulateJava -PhwSim`. The latter provides better output, but will only work if the necessary changes are made to build.gradle, specifically `wpi.sim.addDriverstation().defaultEnabled = true`. 

2 Orange Pi 5 pros with 3 ArduCam OV9782 cameras are used per robot for vision. They run PhotonVision with a custom tag set defined in /generated/AprilTags.json for multi-cam. Multi-cam is only used for robot-to-robot vision, which is a limitation of PV. 

Common hardware issues:
- if azimuths randomly decide to oscillate, restart robot