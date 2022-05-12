# MergingEntities
An interactive AV environment. Movement data gets translated via a NeuralNetwork to drive the soundsynthesis. Movingheads personify a digital entity, that interact with the human body.

A research project at Academy for theatre and digitality: https://theater.digital/#

Produced by ArtesMobiles https://artesmobiles.art

## Software Parts
Get an overview in the flowchart.PDF
* GyropArduino: Arduino (Platformio) Code for gyroscopes
* SoundSynthesis: SuperCollider
* touchdesigner: Tracking, MachineLearning
* MadMapper: Mapping if videoprojection is used
* Light: MagicQ (chamsys) to calibrate the Movingheads

## Machine Learning Install
1. Setup an Andaconda environment with the name "merging"
2. Install Keras or ML framework of your choice
3. Check if your environment was loaded by TouchDesigner:
  * Go in Component "ML" and check if "execute1" compiled correctly
  * open a textport and try to "import Keras"

More infos: https://derivative.ca/community-post/tutorial/anaconda-managing-python-environments-and-3rd-party-libraries-touchdesigner

## Bodysuite / Gyroscopes OSC Interface
* each bodysuite with $id (counting from 1) sends it's gyroscope data like this: /body/$id/gyro/1...6/[quartanians, angles, rotation-speed], where quartanians has 4 dims and angles and rotation-speed has 3 dims x,y,z

## Pharus / LiDAR Tracking Interface
* for each ID there comes an OSC message with: [id, Posx, Posy, Normalx, Normaly, Orientationx, Orientationy, Speed]
