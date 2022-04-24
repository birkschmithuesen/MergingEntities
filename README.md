# MergingEntities
An interactive AV environment. Movement data gets translated via a NeuralNetwork to drive the soundsynthesis. Movingheads personify a digital entity, that interact with the human body.

A research project at Academy for theatre and digitality: https://theater.digital/#

Produced by ArtesMobiles https://artesmobiles.art

## Software Parts
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
