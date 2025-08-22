# FACTR Interface

A bridge between FACTR and ROS2. Includes force feedback and gravity compensation.

## Program flow

1. Init

    a. Declare and read ROS parameters

    b. Create publishers and subscribers

    c. Instantiate `Factr`, which abstracts communication with Dynamixels

    d. Instantiate `Gui`, which shows `Factr`'s current state in a Viser web app

    e. Spin controller, which calculates gravity comp and force feedback and sends commands to both `Factr` and the follower (via JointTrajectory messages)