

<div align="center">
    <img src="https://i.imgur.com/PK5wYK5.png" width="100%"/>
    <br></br>
  <p>
    <a href="https://discord.gg/qCRpgEY"><img src="https://discordapp.com/api/guilds/345404637374971907/embed.png" alt="Discord server" /></a>
  </p>
  <b>Created by Alex Carter of Disnode Robotics</b>
    <br/>
     <i>Version 0.4 Last Updated 12/17/17</i>

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

## Glyph Detector (Working, last Updated: 12/6/17)
This is a detector that uses a mix of filters and canny edge detection that is fed into FindContours. Then each result is scored based on Ratio, Area,
Distance from Bottom-Center of the screen, and soon color. The top scoring result is returned. The value that will be returned inside DogeCV will be a distance
from Center Screen on the X Axis. This can be fed into the bot to tell it which direction to turn.
#### Detector Classes
 - `GlyphDetector` - Cryptobox Detector
#### Parameters
- `downScaleFactor` - double representing how much to downscale each frame. (Lower = Faster) 
- `speed` - Speed setting for the detector. (how fast vs how good)
- `rotateMat` - Rotate the image when processing (wont be visible on preview, change this if you see detections working horizontally) [Usually: Landscape = false, Portrait = true]
- `minScore` - The minimum score for results (threshold)
- `debugDrawStats` - Draw Scores for each result
- `debugDrawRects` - Draw All Found Rectangles

#### Returned Data
Currently This Detector Returns the Following:
- `getChosenGlyphPosition()` - The Position of the Choosen Glyph on the screen (Point)
- `getChosenGlyphOffset()` - The Distance of the chosen glyph from the center of the screen
- `isFoundRect()` - Is there a glyph found?

## Cryptobox Detector (Working, Last Updated: 12/6/17)
This detector finds the position of each column inside the cryptobox. It currently used HSV values to do this so color and lighting will effect it. Im looking
to other ways of doing this. 

Im currently developing a new version of this detector as it is basic and prone to failure, however I decided to release this as it's better the nothing. 

#### Detector Classes
 - `CryptoboxDetector` - Cryptobox Detector

#### Parameters
- `downScaleFactor` - double representing how much to downscale each frame.
- `detectionMode` - Mode used to detect, `HSV_BLUE` and `HSV_RED` are currently only implemented modes, each representing which color you what to detect.
- `speed` - Speed setting for the detector. (how fast vs how good)
- `rotateMat` - Rotate the image when processing (wont be visible on preview, change this if you see detections working horizontally) [Usually: Landscape = false, Portrait = true]
#### Returned Data
Currently This Detector Returns the Following:
- `isCryptoBoxDetected()` - Is the full box detected?
- `isColumnDetected()` - Is at least one column detected?
- `getCryptoBoxLeftPosition()` - Get the left column position (int on x-axis)
- `getCryptoBoxCenterPosition()` - Get the center column position (int on x-axis)
- `getCryptoBoxRightPosition()` - Get the right column position (int on x-axis)
- `getCryptoBoxPositions()` - Array on Ints that represent columns found, in order from left to right

## Jewel Detector (Working, Last Updated: 12/17/17)
This detector find the current orientation of the jewels using color filtering.

#### Detector Classes
 - `JewelDetector` - Cryptobox Detector

#### Parameters
- `downScaleFactor` - double representing how much to downscale each frame.
- `detectionMode` - Mode used to score results, 
  - `MAX_AREA` returns the largest objects in the frame (Good for quick and easy since it doesn't need tuning, but less accurate)
  - `PERFECT_AREA` returns the objects closest to a desired area (Needs tuning, use debugContours to get areas)
- `rotateMat` - Rotate the image when processing (wont be visible on preview, change this if you see detections working horizontally) [Usually: Landscape = false, Portrait = true]
- `areaWeight` - How much does area affect the results (should always be a decimal under 0.1 due to the large numbers returned by areas)
- `perfectArea` - The area of an object (in pixels) that is deemed "perfect" for jewels.
- `debugContours` - Show debug information for contours in the image, including areas.
- `maxDiffrence` - Max diffrence for objects (lower is more restrictive, recommend 10-20)
- `ratioWeight` - How is the ratio of the object's width and height weighted (recommend 10-20)
- `minArea` - Min area for objets (this depends on downScale, but I recommend 500+);

#### Returned Data
Currently This Detector Returns the Following:
- `getLastOrder()` - Get the last known jewel order (`BLUE_RED` or `RED_BLUE`)
- `getCurrentOrder()` -  Get the current jewel order (`BLUE_RED`, `RED_BLUE` or `UNKNOWN`)


## Contact
If you have any suggestions or questions feel free to contact me at:    
**VictoryForPhil@gmail.com**
or 
**VictoryForPhil#4759** on Discord

You can also usually spot me on the FTC Discord.
