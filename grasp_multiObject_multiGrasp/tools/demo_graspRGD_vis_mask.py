#!/usr/bin/env python

# --------------------------------------------------------
# Tensorflow Faster R-CNN
# Licensed under The MIT License [see LICENSE for details]
# Written by Xinlei Chen, based on code from Ross Girshick
# --------------------------------------------------------

"""
Demo script showing detections in sample images.

See README.md for installation instructions before running.
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
import math
import _init_paths
from model.config import cfg
from model.test import im_detect
from model.nms_wrapper import nms

from utils.timer import Timer
import tensorflow as tf
import matplotlib.pyplot as plt
import numpy as np
import os, cv2
import argparse
import glob
from nets.vgg16 import vgg16
from nets.resnet_v1 import resnetv1
import scipy
from shapely.geometry import Polygon
import re

pi     = scipy.pi
dot    = scipy.dot
sin    = scipy.sin
cos    = scipy.cos
ar     = scipy.array

CLASSES = ('__background__',
           'angle_01', 'angle_02', 'angle_03', 'angle_04', 'angle_05',
           'angle_06', 'angle_07', 'angle_08', 'angle_09', 'angle_10',
           'angle_11', 'angle_12', 'angle_13', 'angle_14', 'angle_15',
           'angle_16', 'angle_17', 'angle_18', 'angle_19')

NETS = {'vgg16': ('vgg16_faster_rcnn_iter_70000.ckpt',),'res101': ('res101_faster_rcnn_iter_110000.ckpt',),'res50': ('res50_faster_rcnn_iter_240000.ckpt',)}
DATASETS= {'pascal_voc': ('voc_2007_trainval',),'pascal_voc_0712': ('voc_2007_trainval+voc_2012_trainval',),'grasp': ('train',)}

def Rotate2D(pts,cnt,ang=scipy.pi/4):
    '''pts = {} Rotates points(nx2) about center cnt(2) by angle ang(1) in radian'''
    return dot(pts-cnt,ar([[cos(ang),sin(ang)],[-sin(ang),cos(ang)]]))+cnt



def vis_detections(ax, image_name, im, class_name, dets, thresh=0.5):
    """Draw detected bounding boxes."""

    inds = np.where(dets[:, -1] >= thresh)[0]

    if len(inds) == 0:
        return

    im = im[:, :, (2, 1, 0)]
    #fig, ax = plt.subplots(figsize=(12, 12))
    ax.imshow(im, aspect='equal')
    done = False
    best_score=-1
    best_i=-1
    for i in inds:
        bbox = dets[i, :4]
        score = dets[i, -1]
        if score>best_score:
            best_score=score
            best_i = i
        #ax.add_patch(
        #    plt.Rectangle((bbox[0], bbox[1]),
        #                  bbox[2] - bbox[0],
        #                  bbox[3] - bbox[1], fill=False,
        #                  edgecolor='red', linewidth=3.5)
        #    )

        # plot rotated rectangles
    i=best_i
    bbox = dets[i, :4]
    score = dets[i, -1]
    pts = ar([[bbox[0],bbox[1]], [bbox[2], bbox[1]], [bbox[2], bbox[3]], [bbox[0], bbox[3]]])
    cnt = ar([(bbox[0] + bbox[2])/2, (bbox[1] + bbox[3])/2])
    cx = (bbox[0] + bbox[2])/2
    cy = (bbox[1] + bbox[3])/2
    angle = int(class_name[6:])
    r_bbox = Rotate2D(pts, cnt, -pi/2-pi/20*(angle-1))
    pred_label_polygon = Polygon([(r_bbox[0,0],r_bbox[0,1]), (r_bbox[1,0], r_bbox[1,1]), (r_bbox[2,0], r_bbox[2,1]), (r_bbox[3,0], r_bbox[3,1])])
    pred_x, pred_y = pred_label_polygon.exterior.xy

    plt.plot(pred_x[0:2],pred_y[0:2], color='k', alpha = 0.7, linewidth=1, solid_capstyle='round', zorder=2)
    plt.plot(pred_x[1:3],pred_y[1:3], color='r', alpha = 0.7, linewidth=3, solid_capstyle='round', zorder=2)
    plt.plot(pred_x[2:4],pred_y[2:4], color='k', alpha = 0.7, linewidth=1, solid_capstyle='round', zorder=2)
    plt.plot(pred_x[3:5],pred_y[3:5], color='r', alpha = 0.7, linewidth=3, solid_capstyle='round', zorder=2)
    #print(pred_x,pred_y)
    plt.scatter(cx,cy)
    pred_x.pop(-1)
    pred_y.pop(-1)
    degree_angle = 0
    if pred_x[1] != pred_x[2]:
        slope = (pred_y[2]-pred_y[1])/(pred_x[1]-pred_x[2])
        degree_angle = math.atan(slope)*(180/pi)
    else:
        degree_angle = 90.0
    if degree_angle>=0 :
        degree_angle = 90 - degree_angle
    else :
        degree_angle = -1*(90-abs(degree_angle))
    print(degree_angle,'roll')

    x_co = []
    y_co = []
    for i in pred_x:
        x_co.append(i)
    for i in pred_y:
        y_co.append(i)
    print(x_co,'x-coordinates')
    print(y_co,'y-coordinates')
    done = True
        #ax.text(bbox[0], bbox[1] - 2,
        #        '{:s} {:.3f}'.format(class_name, score),
        #        bbox=dict(facecolor='blue', alpha=0.5),
        #        fontsize=14, color='white')
    return done
    #ax.set_title(('{} detections with '
    #              'p({} | box) >= {:.1f}').format(class_name, class_name,
    #                                              t1hresh),
    #              fontsize=14)
    #plt.axis('off')
    #plt.tight_layout()

    #save result
    #savepath = './data/demo/results/' + str(image_name) + str(class_name) + '.png'
    #plt.savefig(savepath)

    #plt.draw()


def demo(sess, net, image_name, mask_name,number,notfound):
    """Detect object classes in an image using pre-computed object proposals."""

    # Load the demo image
    im = cv2.imread(im_name)
    mask = cv2.imread(mask_name)
    #im = im*np.stack([mask,mask,mask], axis=2)
    im = cv2.bitwise_and(im,mask,mask = None)
    
    # Detect all object classes and regress object bounds
    timer = Timer()
    timer.tic()
    scores, boxes = im_detect(sess, net, im)

    scores_max = scores[:,1:-1].max(axis=1)
    scores_max_idx = np.argmax(scores_max)
    scores = scores[scores_max_idx:scores_max_idx+1,:]
    boxes = boxes[scores_max_idx:scores_max_idx+1, :]

    #im = cv2.imread('/home/fujenchu/projects/deepLearning/tensorflow-finetune-flickr-style-master/data/grasps_ivalab/rgb_cropped320/rgb_0076Cropped320.png')
    timer.toc()
    print('Detection took {:.3f}s for {:d} object proposals'.format(timer.total_time, boxes.shape[0]))

    fig, ax = plt.subplots(figsize=(12, 12))
    # Visualize detections for each class
    CONF_THRESH = 1   
    NMS_THRESH = 0.3
    best_cls=None
    best_score=-1
    best_dets=None
    index_empty = True

    while index_empty:
        CONF_THRESH *= 0.1
        for cls_ind, cls in enumerate(CLASSES[1:]):
            cls_ind += 1 # because we skipped background
            cls_boxes = boxes[:, 4*cls_ind:4*(cls_ind + 1)]
            cls_scores = scores[:, cls_ind]
            dets = np.hstack((cls_boxes,
                            cls_scores[:, np.newaxis])).astype(np.float32)
            keep = nms(dets, NMS_THRESH)
            dets = dets[keep, :]

            try:
                inds = np.where(dets[:, -1] >= CONF_THRESH)[0]
                if len(inds) > 0:
                    index_empty = False
                
                for i in inds:
                    bbox = dets[i, :4]
                    score = dets[i, -1]
                    
                    if(score>best_score) :
                        best_score=score
                        best_cls=cls
                        best_dets=dets
            
            except:
                pass
    #print("Best Score:",best_score,CONF_THRESH)    
    vis_detections(ax, image_name, im, best_cls, best_dets, thresh=CONF_THRESH)
    plt.axis('off')
    plt.tight_layout()

    #cv2.imshow('deepGrasp_top_score', im)
    #choice = cv2.waitKey(100)
    
    #save result
    savepath = os.path.join(cfg.DATA_DIR, 'results', 'result'+number+'.png')
    plt.savefig(savepath)

    plt.draw()

def parse_args():
    """Parse input arguments."""
    parser = argparse.ArgumentParser(description='Tensorflow Faster R-CNN demo')
    parser.add_argument('--net', dest='demo_net', help='Network to use [vgg16 res101]',
                        choices=NETS.keys(), default='res101')
    parser.add_argument('--dataset', dest='dataset', help='Trained dataset [pascal_voc pascal_voc_0712]',
                        choices=DATASETS.keys(), default='pascal_voc_0712')
    args = parser.parse_args()

    return args

if __name__ == '__main__':
    cfg.TEST.HAS_RPN = True  # Use RPN for proposals
    args = parse_args()

    # model path
    demonet = args.demo_net
    dataset = args.dataset
    tfmodel = os.path.join('output', demonet, DATASETS[dataset][0], 'default',
                              NETS[demonet][0])


    if not os.path.isfile(tfmodel + '.meta'):
        raise IOError(('{:s} not found.\nDid you download the proper networks from '
                       'our server and place them properly?').format(tfmodel + '.meta'))

    # set config
    tfconfig = tf.ConfigProto(allow_soft_placement=True)
    tfconfig.gpu_options.allow_growth=True

    # init session
    sess = tf.Session(config=tfconfig)
    # load network
    if demonet == 'vgg16':
        net = vgg16(batch_size=1)
    elif demonet == 'res101':
        net = resnetv1(batch_size=1, num_layers=101)
    elif demonet == 'res50':
        net = resnetv1(batch_size=1, num_layers=50)
    else:
        raise NotImplementedError
    net.create_architecture(sess, "TEST", 20,
                          tag='default', anchor_scales=[8, 16, 32])
    saver = tf.train.Saver()
    saver.restore(sess, tfmodel)

    print('Loaded network {:s}'.format(tfmodel))

    #im_names = ['rgd_0076Cropped320.png','rgd_0095.png','pcd0122r_rgd_preprocessed_1.png','pcd0875r_rgd_preprocessed_1.png','resized_0875_2.png']
    im_names = glob.glob(os.path.join(cfg.DATA_DIR, 'demo', '*.png')) 
    mask_names = glob.glob(os.path.join(cfg.DATA_DIR,'..', 'tools','masks', '*.jpg'))
    mask_default_path = os.path.join(cfg.DATA_DIR,'..', 'tools','masks', 'mask_default.jpg')
    for im_name in im_names:
        print('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
        print('Demo for data/demo/{}'.format(im_name))
        numbers = re.findall('\d+',im_name)
        numbers = map(int,numbers)
        number = max(numbers)
        mask = [s for s in mask_names if (str(number)+'.jpg') in s]
        if len(mask)==0:
          demo(sess, net, im_name, mask_default_path,str(number),True)
          continue
        demo(sess, net, im_name, mask[0],str(number),False)

    plt.show()
