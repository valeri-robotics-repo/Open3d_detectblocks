# Open3d_example


## Examples
- [ROS2 Usage Example](https://github.com/ROS2-block-detection#readme) - Project that uses this library in ROS2, step-by-step installing instructions.

-  For a demonstration of my robot performing block detection using this library and stacking blocks, visit my website here:
-    http://edwardtherobot.com


How to run this project:
    1.  Install Open3d using the instructions provided for C++  (Python is not required):
        http://www.open3d.org/docs/release/compilation.html

    2.  In order to run the tests, you will need to have GTEST installed:
           sudo apt-get install libgtest-dev
    
    3.  I use Visual Studio Code to build, debug and run this project.  Follow the steps to install your kit.
    
    4.  When ready, you will need to run the install to have access to this project from your ROS2 build, assuming you have it in a separate project.
        In the sharedlib folder,  run the following:

