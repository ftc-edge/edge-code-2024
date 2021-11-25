## Welcome!
This repository is a fork of the official FTC SDK from https://github.com/FIRST-Tech-Challenge/FtcRobotController. It has the addition of our Titan Robotics Framework Library (TrcLib) and its dependencies. These are added as git submodules so that they can be independently managed without the need of changing the FTC SDK (except for minor gradle changes to tie in our Framework Library dependencies). Therefore, one can update the FTC SDK by pulling changes from the upstream FTC SDK without affecting our Framework Library. You can also pick up the latest changes of our Framework Library without affecting the FTC SDK. It serves as a clean repository template when the season starts. This template also contains basic team code that implements a mecanum drive base with teleop control. It allows you to run a simple mecanum robot in TeleOp almost right out of the box. In addition, it includes autonomous infrastructure code that uses our FtcChoiceMenu for selecting autonomous strategies and other autonomous choice opotions. This allows you to write only a few autonomous opmodes to handle many permutations of autonomous. For example, one autonomous opmode can handle both red and blue alliances. The template also provides a rich set of Tests for diagnosing robot hardware and/or for tuning.

## Getting Started
To use this repository template, you can fork this repository to your own github repositories. There are many ways to do it but I am going to describe one way which is using GitHub Desktop. If you are more familiar with other similar tools, feel free to use it instead.
* On a web browser, enter the URL https://github.com/trc492/FtcTemplate.
* Then, click the "fork" button on the upper right corner of the web page and answer the questions on where you want to fork this repository.
* Once the fork is done. Clone your forked repository to your computer using your favorite GitHub tool such as GitHub Desktop.
* On GitHub Desktop, click File->Clone repository..., select the repository you just forked and type in the path on the computer where you want to clone the repository to.

Congratulations! You just clone a fork of our template repository. Now you can fire up Android Studio and import this gradle project. Once that is done, you can now go to TeamCode and browse around the provided template code. You can compile the code and check if you have any issues with the cloned template. Or you can jump right in and start modifying/customizing the code.

## Getting Help
### User Documentation and Tutorials
We are very bad at creating documentation and tutorials but we want to get better at this. Our Framework Library code has JavaDoc all over. Therefore, you can get information on what each class do and their methods. We will add in some sample code to the project soon. We will also try to monitor the "FIRST Tech CHALLENGE" Discord forums. Several teams have been using our Framework Library and we welcome opportunities of collaboration in creating tutorial materials. I will try to create some JavaDocs and put out a link to it in the near future. Feel free to suggest what tutorial you want to see.

### Javadoc Reference Material
The Javadoc reference documentation for the TRC Robotics Framework Library can be found [here](https://github.com/trc492/FtcTemplate/FtcJavaDoc/index.html).

### Online User Forum
For technical questions regarding our Framework Library, please visit the FIRST Tech CHALLENGE Discord forum.

### Sample OpModes

# Release Information

## Version 1.0 (2021-11-24)

### Enhancements and New Features

### Bug fixes

