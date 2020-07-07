import numpy as np    
import imageio as i
import os
import glob
import re

txt_names = glob.glob(os.path.join('./output/'+'*.txt')) 
#print(txt_names)
for t in txt_names:
  f = open(t)
  print(t)
  lines = [line.rstrip() for line in f]
  #print(lines)
  mask = np.zeros((480,640))
  bboxes=0
  for line in lines:
    co = line.split()
    #print(co)
    x1 = int(co[0])
    y1 = int(co[1])
    x2 = int(co[2])
    y2 = int(co[3])
    mask[y1:y2,x1:x2]=1
    bboxes=bboxes+1
    if bboxes==5:
      break
  numbers = re.findall('\d+',t)
  numbers = map(int,numbers)
  number = max(numbers)
  print(number)
  i.imwrite('masks/mask'+str(number)+'.jpg', mask)

mask = np.ones((480,640))
i.imwrite('masks/mask_default.jpg', mask)