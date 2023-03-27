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
TouchDesigner will always use its shipped cuDNN file, no matter which conda environment is loaded. Therefore we need to build the system around this cuDNN file -> cuDNN 8.1.1
It was not able to include a matching Cudatoolkit and cuDNN file in the environment therefore this needs to be downloaded and installed manually on Windows level. 
* Step 1: Download and install CUDA Toolkit v11.2.2: https://developer.nvidia.com/cuda-11-2-2-download-archive (you only need the runtime libraries, selectable through custom install)
* Step 2: Download cuDNN v8.1.1: https://developer.nvidia.com/compute/machine-learning/cudnn/secure/8.1.1.33/11.2_20210301/cudnn-11.2-windows-x64-v8.1.1.33.zip
* Step 3: Copy/Paste the folders bin, include and lib from the cuDNN folder into the CUDA install folder “C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v11.2”. (replace files if asked)
* Step 4: Add "C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v11.2\bin" to path environment variables
* Step 5: Load Anaconda environment from touchdesigner/Conda/td_merging-entities.yml


## Bodysuite / Gyroscopes OSC Interface
Each bodysuite with $id (counting from 1) sends it's gyroscope data like this: `/body/$id/gyro/$part/[quarternions, angles, rotation-speed]`.
The values for $part are: `left_arm`, `right_arm`, `left_foot`, `right_foot`, `back`, and `head`.
The quaternions have 4 float values, the angles and angles and rotation-speed have 3 float values for x,y,z.

## Pharus / LiDAR Tracking Interface
* for each ID there comes an OSC message with: [id, Posx, Posy, Normalx, Normaly, Orientationx, Orientationy, Speed]
