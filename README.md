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
Setup an Andaconda environment from the requirements file /touchdesigner/environment.yml with "conda env create -f environment.yml -n MY-NAME"
More infos: https://derivative.ca/community-post/tutorial/anaconda-managing-python-environments-and-3rd-party-libraries-touchdesigner
If wanted install CUDA -> CUDA version 11.5
* CUDA Toolkit: https://www.dropbox.com/s/qid60lpnmoyj46p/cuda_11.5.0_496.13_win10.exe?dl=0
* CUDNN Files: https://www.dropbox.com/s/hufr0ot3kak5hbi/cudnn-windows-x86_64-8.3.3.40_cuda11.5-archive.zip?dl=0
* Follow Tutorial: https://www.youtube.com/watch?v=OEFKlRSd8Ic


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
