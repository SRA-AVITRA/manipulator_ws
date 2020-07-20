# grasp_multiObject_multiGrasp

This is a combination of many repos integrated together to perform the task of pose estimation.
```
Input : RGD & RGB image 
Output : Single grasp in the frame of camera. 
```

## Difference between perception branch and updated_perception branch:

1. Object detection was not used in getting output in perception branch. Modifications were made by the Computer Vision team later on by adding Object Detection to improve performance of the model. However, these changes could not be merged with testing with the perception branch later on due to lockdown condition. So, a separate branch exists namely updated_perception containing these changes. Same trained weights can be used in both branch source codes.   

2. The source code to run in perception branch is manipulator_ws/grasp_multiObject_multiGrasp/tools/demo_graspRGD.py. A bash file is created in updated_perception branch namely 
getGrasp.sh. It contains instructions to run Yolo source code followed by generating mask for detected objects and finally grasp detection is done by demo_graspRGD_vis_mask.py source code.

3. Notice that in demo_graspRGD.py of perception branch there is a boolean variable single. If single is set to true then only one bounding box which is of highest score will be given at output. If it is set to false then all possible bounding boxes in entire image will be shown at output. This variable option was removed in new branch demo_graspRGD_vis_mask.py source code as bounding box on only single object was required and it can be again changed to obtain more than one bounding box. 

4. In order to train grasp detection model, perception branch cannot be used. Reason: changes were made in setup.py source code so as to make the code run on machine not having GPU (NVC). So, for training purpose use the updated_perception branch. 

So, the updated_perception branch should be used to train new grasp detection model. But for testing part, the changes made such as Yolo with grasp detection should be merged to the perception branch.    

# Requirements
- Tensorflow 1.x
- Python 3
- Matlab/Octave (For data pre-processing)


# Demo

1. Clone this repository

```bash
git clone <link of current repository>
cd grasp_multiObject_multiGrasp
```

2. Build Cython modules

```bash
cd lib
make clean
make
cd ..
```

3. Install [Python COCO API](https://github.com/cocodataset/cocoapi)

```bash
cd data
git clone https://github.com/pdollar/coco.git
cd coco/PythonAPI
make
cd ../../..
```

4. Download pretrained models
- trained model for grasp on [dropbox drive](https://www.dropbox.com/sh/ybwwbj7zzwxxrvw/AAD25r-w08xx9FYKm_pllgf-a?dl=0)
- put under `output/res50/train/default/`


5. Run demo
- Put rgb images in `data/demo_color`
- Put rgd images in `data/demo`
- Make sure `tools/mask` & `tools/output` is empty
- Run `./getGrasp.sh`


# Training 

## Preprocessing 

Cornell Dataset contains RGB and Depth images of many objects with labels for suitable grasp regions in an image.

Cornell Dataset can be downloaded from the following website:

http://pr.cs.cornell.edu/grasping/rect_data/data.php


Steps 1 and 2 are transformations specific to Cornell images. Rest of the steps are common to both Cornell and Real Sense images.

### 1. PCD to PNG format conversion

Source code: `pcdToDepth.py`. The Cornell dataset contains RGB and Depth information. However, the depth information is stored in pcd(Point Cloud Data) format. To convert PCD to PNG image format. 

### 2. Negative Transformation of Cornell Images

The depth image of Cornell dataset is not consistent with the depth image of RealSense camera.Negative transformation of Cornell images should be performed to make it consistent with RealSense. cv2.bitwise_not() function of OpenCV was applied on Cornell image for the same.

### 3. Converting RGB and Depth Images to RGD Image

Source code: `rgd.py`. The GraspDetection model is trained on RGD channels of image.So the B(blue) channel is replaced by D(depth) to get RGD images.

### 4. Splitting dataset into train and test folders

Source code: `dataPreprocessingTest_fasterrcnn_split.m`. Data augmentation (rotate,translate) will be performed and data will be split into train and test folder. 


Following directory structure should be created beforehand.
```
Data
|____ Images
|____ Annotations
|____ ImageSets
      |_____ train.txt
      |_____ test.txt
```

After running dataPreprocessingTest_fasterrcnn_split.m,above folders will be filled with images and text files as needed.


## Train

Set graspRGB_devkit_path in `lib/datasets/factory.py` source code to the absolute path of the repo.

Execute following command:
```
./experiments/scripts/train_faster_rcnn.sh 0 graspRGB res50
```

### Code Structure
```
grasp_multiObject_multiGrasp #Main repo for testing and training
├── data #Subdirectory for image data,preprocessing & dataset creation 
│   ├── coco #COCO API (README for more info)
│   │   ├── common 
│   │   ├── license.txt
│   │   ├── LuaAPI
│   │   ├── MatlabAPI
│   │   ├── PythonAPI
│   │   ├── README.txt
│   │   └── results
│   ├── demo #Place testing images (RGD format) here
│   ├── demo_color #Place testing images (RGB format) here
│   ├── results #Result images will be saved here
│   └── preprocessing #Scripts for dataset before training
│       ├── dataPreprocessing_fasterrcnn.m #Data Augmentation
│       ├── dataPreprocessingTest_fasterrcnn_split.m #Testing
│       |── fetch_faster_rcnn_models.sh #Download pretrained weights
|       |__ pcdToDepth.py
|       |__ rgd.py
├── experiments #Code to train a new model 
│   ├── cfgs #Network configurations
│   ├── logs
│   └── scripts
├── getGrasp.sh #Run for demo
├── lib #Helper and support code
│   ├── datasets #Dataset abstraction code
│   ├── layer_utils #Utility functions for network
│   ├── Makefile
│   ├── model #Model helper functions
│   ├── nets #Network layer definitions
│   ├── nms #Cpu and cuda nms
│   ├── roi_data_layer
│   ├── setup.py # Installs all dependencies 
│   ├── setup.py~
│   └── utils #Misc utility
├── output
│   └── res50
│       └── train
│           └── default # The weight files
│               ├── res50_faster_rcnn_iter_240000.ckpt.data-00000-of-00001
│               ├── res50_faster_rcnn_iter_240000.ckpt.index
│               ├── res50_faster_rcnn_iter_240000.ckpt.meta
│               └── res50_faster_rcnn_iter_240000.pkl
├── README.md
├── tools #High level implementation of grasp detection
│   ├── demo_graspRGD.py #Ordinary grasp detect
│   ├── demo_graspRGD.py~
│   ├── demo_graspRGD_socket_drawer.py~
│   ├── demo_graspRGD_socket.py~
│   ├── demo_graspRGD_socket_save_to_rgbd.py~
│   ├── demo_graspRGD_vis_mask.py #Grasp detection with masking
│   ├── demo_graspRGD_vis_select.py #Grasp detection with selection
│   ├── demo.py~
│   ├── eval_graspRGD.py~
│   ├── _init_paths.py
│   ├── _init_paths.pyc
│   ├── mask_gen.py #Create mask with co-ordinates from output
│   ├── masks #Mask images are stored here
│   ├── output #Yolo visualization and txt co-ordinates stored here
│   ├── __pycache__
│   └── trainval_net.py 
└── yolov3 #Object detection code
    ├── cfg #Yolo config files
    ├── data #Coco dataset files
    ├── detect.py #Detection code
    ├── Dockerfile
    ├── examples.ipynb
    ├── LICENSE
    ├── models.py 
    ├── __pycache__
    ├── README.md
    ├── requirements.txt #PIP install all dependencies
    ├── test.py #Testing code
    ├── train.py #Training code
    ├── utils #Utility code
    └── weights #Weight download script

48 directories, 202 files
```

## Acknowledgment
This repo borrows tons of code from 
- grasp_multiObject_multiGrasp by ivalab
- yolov3 by ultralytics
- tf-faster-rcnn by endernewton
