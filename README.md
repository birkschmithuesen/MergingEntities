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

## Anaconda Environment
* Name: merging-entities
* Python: 3.7.2
* Requirements in requirements.txt in touchdesigner folder
* Environment in environment.yml in touchdesigner folder
We encountered unsolveable Crashes when fitting LSTM models. Research showed that it's not better to use 'conda install' instead of 'pip install' for this problem. We could verify this, installing envs with pip packages crashed the code, conda packages were running. The packages install via conda or pip also installed different versions of the packages (conda: tensorflow 3.2, pip: tensorflow 3.9).
EDIT: It seems after de-installing CUDA the problem disappears. This could be the result of unmatchin CUDA and PY and TD versionsm but for now no CUDA anymore. 

## Machine Learning Install
Setup an Andaconda environment from the requirements file /touchdesigner/environment.yml with "conda env create -f environment.yml -n MY-NAME"


More infos: https://derivative.ca/community-post/tutorial/anaconda-managing-python-environments-and-3rd-party-libraries-touchdesigner

## Bodysuite / Gyroscopes OSC Interface
* each bodysuite with $id (counting from 1) sends it's gyroscope data like this: /body/$id/gyro/1...6/[quartanians, angles, rotation-speed], where quartanians has 4 dims and angles and rotation-speed has 3 dims x,y,z
1 - left arm
2 - right arm
3 - left foot
4 - right foot
5 - back
6 - head

## Pharus / LiDAR Tracking Interface
* for each ID there comes an OSC message with: [id, Posx, Posy, Normalx, Normaly, Orientationx, Orientationy, Speed]
