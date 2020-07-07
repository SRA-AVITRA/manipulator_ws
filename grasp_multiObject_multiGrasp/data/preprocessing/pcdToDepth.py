import os
import math
import numpy as np
from PIL import Image
import glob

path = os.path.join('pcd*[0-9].txt')
print(path)
pcds = glob.glob(path)
pcds.sort()
count = 1
#Function to normalize the value
def normalize(x):
    return 255 * (x - min_d) / max_min_diff
#reading data from pcd file,setting the resolution,col and row give the data points finally calling the normalize function and saving the images
for pcd in pcds:
    with open(pcd) as pcd_file:
        lines = [line.strip().split(" ") for line in pcd_file.readlines()]
    img_height = 480
    img_width = 640
    is_data = False
    min_d = 0
    max_d = 0
    img_depth = np.zeros((img_height, img_width), dtype='f8')
    for line in lines:
        if line[0] == 'DATA':
            is_data = True
            continue
        if is_data:
            d = max(0., float(line[2]))
            i = int(line[4])
            col = i % img_width
            row = math.floor(i / img_width)
            img_depth[row, col] = d
            min_d = min(d, min_d)
            max_d = max(d, max_d)
    max_min_diff = max_d - min_d
    
    normalize = np.vectorize(normalize, otypes=[np.float])
    img_depth = normalize(img_depth)
    img_depth_file = Image.fromarray(img_depth)
    of_name = pcd.replace('.txt', 'depth_image.png')
    img_depth_file.convert('RGB').save(of_name)
    print(count)
    count=count+1

