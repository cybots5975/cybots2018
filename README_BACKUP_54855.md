[![Discord](https://img.shields.io/discord/377144270034829324.svg?style=for-the-badge)](https://discord.gg/ameFTnC)

# OpenFTC-app-turbo is to be used at your own risk.
##### While every change is made with care not to break anything, FIRST has NOT tested our modifications.
##### Volunteers at competitions may be less willing to assist you if you use OpenFTC. Our community is very helpful though, so please create an issue or talk to us on Discord if you encounter any problems.

<<<<<<< HEAD
[Sign up for our mailing list](https://goo.gl/forms/xFBIx0Ptk3Br7ZOD2) for notifications about updates and upcoming new projects. 

[Join us on Discord](https://discord.gg/Q3CgrxU) for real-time support or to ask questions.

OpenFTC-app is currently based on version 3.5 of the SDK. Do not attempt to manually update beyond that yourself. OpenFTC will be updated within a week of new official app releases.

The OpenFTC documentation is still under construction.

---

OpenFTC-app-turbo removes the OnBotJava and Blocks programming systems for teams that don't use them, speeding up deploy time drastically. If you use these systems, please see [OpenFTC-app](https://github.com/OpenFTC/OpenFTC-app).

The OpenFTC family of Robot Controller apps are based on the official
[FTC SDK](https://github.com/ftctechnh/ftc_app) (Software Development Kit), but the AAR files it uses have been converted to modules in the Android Studio project. This makes it easy to see and modify almost the entirety of the SDK's source code. In addition, the history in Git shows all changes that have been made to the core code since OpenFTC's inception. This is a very useful supplement to the changelogs that FIRST provides - teams can see exactly what code has been changed and how it will affect them.

This system allows pull requests and enhancements to the code of the entire SDK, and can allow teams to understand the structure and functionality of the whole system. Most enhancements will likely be accepted as long as they _do not force teams to change their workflow._ Changes made in the OpenFTC SDK should allow teams to move from the official SDK to OpenFTC-app with no code changes required.

To request a new feature, you can open an issue on this repository. If there's a large enough call for the feature, it's very likely to be added to the list for a future release. 

This version keeps the OnBotJava and Blocks programming system. If your team doesn't use these, you may want to look into [OpenFTC-app-turbo](https://github.com/OpenFTC/OpenFTC-app-turbo), which helps to decrease deploy times. 

# Release Notes
To see the release notes for FIRST's releases of this SDK, see [doc/FIRST_CHANGELOG.md](doc/FIRST_CHANGELOG.md)

## 1.1
* Updated the app to be based on version 3.5 of the official SDK.

## Pre-1.1
* Extracted the aar files and associated source jars into full, easily changed source code modules.
* Warn user if the driver station app is installed on the device
* Fixed bug with log viewer colorization
* Add option that allows the user to choose whether lines should wrap in the log viewer (they don't by default now)
* Changed the icon to make the app more easily distinguishable from the official one
* Added the OpenFTC version to the main screen
* Added "About OpenFTC" button to the about screen
* Allowed Gradle to use more RAM (up to 2GB) for faster build times
##### &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Turbo-specific changes
* Initial Turbo release that removes the Blocks and OnBotJava features


## Welcome!
This GitHub repository contains the source code that is used to build an Android app to control a *FIRST* Tech Challenge competition robot.  To use this SDK, download/clone the entire project to your local computer.

If you are new to the *FIRST* Tech Challenge software and control system, you should visit the online wiki to learn how to install, configure, and use the software and control system:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;https://github.com/ftctechnh/ftc_app/wiki

Note that the wiki is an "evergreen" document that is constantly being updated and edited.  It contains the most current information about the *FIRST* Tech Challenge software and control system.

## Downloading the Project
It is important to note that this repository is large and can take a long time and use a lot of space to download. If you would like to save time and space, there are some options that you can choose to download only the most current version of the Android project folder:

* If you are a git user, *FIRST* recommends that you use the --depth command line argument to only clone the most current version of the repository:

<p>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;git clone --depth=1 https://github.com/ftctechnh/ftc_app.git</p>

* Or, if you prefer, you can use the "Download Zip" button available through the main repository page.  Downloading the project as a .ZIP file will keep the size of the download manageable.

* You can also download the project folder (as a .zip or .tar.gz archive file) from the Downloads subsection of the Releases page for this repository.

Once you have downloaded and uncompressed (if needed) your folder, you can use Android Studio to import the folder  ("Import project (Eclipse ADT, Gradle, etc.)").

## Getting Help
### User Documentation and Tutorials
*FIRST* maintains an online wiki with information and tutorials on how to use the *FIRST* Tech Challenge software and robot control system.  You can access the wiki at the following address:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;https://github.com/ftctechnh/ftc_app/wiki

### Javadoc Reference Material
The Javadoc reference documentation for the FTC SDK is now available online.  Visit the following URL to view the FTC SDK documentation as a live website:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;http://ftctechnh.github.io/ftc_app/doc/javadoc/index.html    

Documentation for the FTC SDK is also included with this repository.  There is a subfolder called "doc" which contains several subfolders:

 * The folder "apk" contains the .apk files for the FTC Driver Station and FTC Robot Controller apps.
 * The folder "javadoc" contains the JavaDoc user documentation for the FTC SDK.

### Online User Forum
For technical questions regarding the SDK, please visit the FTC Technology forum:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;http://ftcforum.usfirst.org/forumdisplay.php?156-FTC-Technology
=======
<div align="center">
    <img src="https://i.imgur.com/sjEURcc.png" width="100%"/>
    <br></br>
  <p>
    <a href="https://discord.gg/qCRpgEY"><img src="https://discordapp.com/api/guilds/345404637374971907/embed.png" alt="Discord server" /></a>
  </p>
  <b>Created by Alex Carter of Disnode Robotics</b>
    <br/>
     <i>Version 1.1.1 Last Updated 1/3/2018</i>

</div>

# DogeCV
A easy to use computer vision library used for FTC Games to detect game objects. Based on Ender CV and OpenCV. 


# DISCLAIMER
### THIS REPO IS STILL UNDER HEAVY DEVELOPMENT. I WILL BE ADDING FURTHER DOCUMENTATION, BUG FIXES AND NEW DETECTORS SOON. MANY OF THE 
However, although many of the detectors are currently pretty basic, I am putting alot of time in effort into this lib, and open sourced it to let the community work or learn from my mistakes. This is the exact code my team will be running so I do have a decent motivation to work on it ;)

## Credits
- Levi 8148 AlephBots (Amazing color filtering and Cryptobox Detector)
- Karter FTC 5975 Cybots (Brainstorming for Jewel Detector)
- Aparna ig (Testing)
- Everyone else on the Programming Discord <3 (Dealing with my bs)

## Known Issues
- Blue in background throws of Crypto checkers. (NO JEANS PLZZZ)
- Gylph Detections pretty buggy

## Planned Features / TODO
- Glyph Color Reading
- Previous frame's results to increase accuracy in detectors
- Motion Tracking for Cryptobox
- Cryptobox line checks follow lines instead of Y-Axis


## Install (Credit to EnderCV)
1. Download this repo, either by cloning from Git or using the zip download. 
2. Open up your FTC Application
3. Navigate to **File** -> **New** -> **Import Module** from the title bar.
4. When the a dialog comes up, asking for the module source directory, navigate to this repo and select the **openCVLibrary320** folder, and then hit **Finish**
5. Repeat steps 3 and 4 except instead of selecting the **openCVLibrary320** folder, select the **DogeCV** folder instead.
6. In the left hand side project explorer in Android Studio, right-click **TeamCode**, and click on **Open Module Settings**.
7. A **Project Struture** dialog should come up. Click the **Dependencies** tab.
8. Click the green plus sign on the right hand side, then **Module dependency**, and then **:openCVLibrary320**, then press OK.
9. Repeat step 8, except substitute **:openCVLibrary320** with **:dogecv**
10. Click **OK** to exit the **Project Structure** dialog.


# Detectors
**See Wiki**

## Changelogs
**1.1.1 HOTFIX**:
 - Fixed Jewel Detector Blue Filter
 

**1.1**:
- New Color Filter API   
- New Generic Detector     
- Fixed Jewel Debug Scores    
- Fixed Imports for DogeLogger inside Cryptobox Detector   
- Ported all detectors to Color Filter API     
- Added Yellow to LeviColorFilter
- Added HSV color filter
- New Relic/Generic Example
 
**1.0**:
 - New Cryptobox Detector
 - Youtube Tutorials
 - Per-Detector Documentation
 - Wiki Start 
 - Added `perfectRatio` tuning for Jewels
 - Optimiziation
 - Removed Multiple Mat returning

**0.5**:
 - Fixed rotated preview on protrait mode.
 - Detectors return an array of images. You can cycle through them by tapping on the preview screen
## Contact
If you have any suggestions or questions feel free to contact me at:    
**VictoryForPhil@gmail.com**
or 
**VictoryForPhil#4759** on Discord

You can also usually spot me on the FTC Discord.
>>>>>>> 3f05a85302f936073acfa9d9bb38f10a021e606e
