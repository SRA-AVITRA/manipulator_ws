cd yolov3/ 
python3 detect.py --save-txt --output "../tools/output" --source "../data/demo_color" --conf-thres 0.1 --classes 39 41 45 40 75 
cd ..
cd tools/
python3 mask_gen.py
cd ..
./tools/demo_graspRGD_vis_mask.py --net res50 --dataset grasp
