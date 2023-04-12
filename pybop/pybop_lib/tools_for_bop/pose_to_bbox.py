#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Author: Thijs van der Burg
# 2023 Delft University of Technology

import datetime
import os

import rospy
import tf2_ros
import tf
from geometry_msgs.msg import PointStamped, TransformStamped
import numpy as np
import cv2
import time

from pybop_lib.transform_tools import get_affine_transform

# import copy

DESC = """
Takes camera to object transform and outputs the 2D bounding box in the image frame
"""

class PoseToBbox(object):
    def __init__(self,padding_ratio=1.5,resize_method="crop_square_resize", GT=False, visualise=False, debug=False, test_limits=False):
        print('initialising PoseToBbox...')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        #TODO make intrinsics parseable, possibly from zed node
        self.intrinsics = np.array([    [521.666748046875,   0.0,   631.2218017578125],
                                        [0.0,   521.666748046875,   363.2084045410156],
                                        [0.0,                0.0,   1.0              ]    ])
        self.box_dims = np.array([0.49, 0.33, 0.365]) # width, depth, height https://tud365-my.sharepoint.com/:f:/r/personal/pmvanderburg_tudelft_nl/Documents/Thesis/datasets/Husky_dataset/Optitrack%20calibration/Marker%20measurements/box?csf=1&web=1&e=aCK5mI
        self.padding_ratio = padding_ratio
        self.resize_method = resize_method
        self.crop_size_img = 256
        self.crop_size_gt = 128
        self.visualise = visualise
        self.GT=GT
        self.debug=debug
        self.test_limits=test_limits
        if self.test_limits:
            self.limit_mode = 1
            self.u_lim, self.v_lim = self.get_limit_params()
        self.stamp = rospy.Time()

    def get_limit_params(self):
        num_bits = 3
        num = self.limit_mode

        bits = [(num >> bit) & 1 for bit in range(num_bits - 1, -1, -1)]
        # print('bits', bits)
        # ulim = 0
        # vlim = 0

        if bits[0]:
            ulim = 100
        else:
            ulim = 0
        if bits[1]:
            vlim = 100
        else:
            vlim = 0
        if bits[2]:
            ulim = -ulim
        else:
            vlim = -vlim
            
        return ulim, vlim

    def bboxToImage(self,_img,_bbox,_savepath):
        '''
        @param:
            - img: dict containing:
                            'image' with 720 x 1280 x 4 numpy array containing the latest image
                            'timestamp' the timestamp in ROS Time
            - bbox: the [ x y width height] bounding box of the object in the image
        @return u,v: pixel coordinate locations corresponding to the world coordinates
        '''
        if self.debug:
            print('bbox to image')
        x_ = _img[:,:,:3]

        
        _bbox_padded = self.padding_Bbox(_bbox,self.padding_ratio)
        # print('_bbox',_bbox)
        
        # Display the image
        # if x_[:,:,:].any():
        if _bbox_padded.any() and ( _bbox_padded[2] < 600 or _bbox_padded[3] < 600 ):    
            # get region of interest from padded bbox
            roi_x = get_roi(x_, _bbox_padded, self.crop_size_img, interpolation=cv2.INTER_LINEAR, resize_method = self.resize_method)
            
            # resize
            Bbox = self.get_final_Bbox(_bbox_padded, self.resize_method, x_.shape[1], x_.shape[0])
            
            # Visualise, convert the 3D RGB array to an RGB OpenCV image
            # rgb_image = cv2.cvtColor(x_, cv2.COLOR_RGBA2RGB)    
            # cv2.imwrite(_savepath,rgb_image)
            
            # Create a numpy array with RGB color values
            # rgb_array = np.zeros((720, 1280, 3), dtype=np.uint8)
            # rgb_array[..., 0] = 255 # Set red channel to 255
            # rgb_array[..., 1] = 255 # Set green channel to 255

            # Add an alpha channel with value 255
            # rgba_array = np.zeros((720, 1280, 4), dtype=np.uint8)
            # rgba_array[..., :3] = rgb_array
            # rgba_array[..., 3] = 255

            # Save the numpy array as a PNG image
            # img_path = _savepath
            # print('img_path',img_path)
            # cv2.imwrite(img_path, rgba_array)

            if self.visualise:
                rgb_image = cv2.cvtColor(x_, cv2.COLOR_RGBA2RGB)
                pre_padd_bbox = _bbox.astype(int)
                cv2.rectangle(rgb_image,(pre_padd_bbox[0],pre_padd_bbox[1]),(pre_padd_bbox[0]+pre_padd_bbox[2] ,pre_padd_bbox[1]+pre_padd_bbox[3] ),(0,255,0),3) 
                # upperleft = (_bbox[0], _bbox[1])
                # bottomright = (_bbox[0]+_bbox[2], _bbox[1]+_bbox[3])
                # cv2.rectangle(rgb_image,( Bbox[0], Bbox[1]),( Bbox[0]+ Bbox[2] , Bbox[1]+ Bbox[3] ),(0,255,0),3) 
                cv2.imwrite(_savepath, rgb_image)        


                # cv2.namedWindow('rgb', cv2.WINDOW_NORMAL)
                # rgb_image = cv2.cvtColor(x_, cv2.COLOR_RGBA2RGB)    
                # cv2.imshow('rgb', rgb_image)
                # cv2.namedWindow('roi', cv2.WINDOW_NORMAL)
                # cv2.imshow('roi', roi_x)
                # cv2.waitKey(1)
            
            return roi_x, Bbox
            
        else:
            print('bbox not any or outside range')
            return np.zeros((self.crop_size_img,self.crop_size_img,3)), np.zeros_like(_bbox_padded)
        
            
    def padding_Bbox(self, Bbox, padding_ratio):
        # get pixel values of four BBox corners
        x1 = Bbox[0]
        x2 = Bbox[0] + Bbox[2]
        y1 = Bbox[1]
        y2 = Bbox[1] + Bbox[3]
        # get center of BBox in x and y dir
        cx = 0.5 * (x1 + x2) # center in x dir (to the right from topleft of img)
        cy = 0.5 * (y1 + y2) # center in y dir (to the bottom from topleft of img)

        bw = Bbox[2]
        bh = Bbox[3]
        
        padded_bw = int(bw * padding_ratio)
        padded_bh = int(bh * padding_ratio)
        # calculate array containing the upperleft BBox corner x and y vals 
        # and the width and height of the padded BBox
        padded_Box = np.array([int(cx-padded_bw/2), int(cy-padded_bh/2), int(padded_bw), int(padded_bh)])
        return padded_Box

    def get_final_Bbox(self, Bbox, resize_method, max_x, max_y):
        # Bbox = self.get_final_Bbox(Bbox (padded), self.resize_method, x.shape[1] imgwidth, x.shape[0] imgheight)
        # get upper left corner x1 y1
        # calc lower right corner
        x1 = Bbox[0]
        bw = Bbox[2]
        x2 = Bbox[0] + bw
        y1 = Bbox[1]
        bh = Bbox[3]
        y2 = Bbox[1] + bh
        if resize_method == "crop_square_resize" or resize_method == "crop_resize_by_warp_affine":
            bbox_center = np.array([0.5 * (x1 + x2), 0.5 * (y1 + y2)])
            if bh > bw:
                # if the height exceeds the width, crop from the width and make the width equal to the height
                x1 = bbox_center[0] - bh/2
                x2 = bbox_center[0] + bh/2
            else: #else, make the height equal to the width
                y1 = bbox_center[1] - bw/2
                y2 = bbox_center[1] + bw/2
            x1 = int(x1)
            y1 = int(y1)
            x2 = int(x2)
            y2 = int(y2)
            Bbox = np.array([x1, y1, x2-x1, y2-y1]) # upperleftxy, width, height

        elif resize_method == "crop_resize":
            x1 = max(x1, 0)
            y1 = max(y1, 0)
            x2 = min(x2, max_x)
            y2 = min(y2, max_y)
            x1 = int(x1)
            y1 = int(y1)
            x2 = int(x2)
            y2 = int(y2)
            Bbox = np.array([x1, y1, x2-x1, y2-y1])

        return Bbox

    def pose_to_Bbox(self,stamp): #,pose_ObjToCam): #, latestcommontime):
        # intrins = self.intrinsics
        # width = self.box_dims[0]
        # depth = self.box_dims[1]
        # height = self.box_dims[2]
        # get corners 3D points from camera frame
        # self.stamp = stamp
        points = self.getCornerPoints_(stamp)  #pose_ObjToCam) #,latestcommontime)
        if not points[0].point.x:
            # print('pose to bbox points 0 point x is zero, making bbox zero')
            bbox = np.array([0,0,0,0])
            bbox_stamp = points[0].header.stamp
            # if self.debug:
                # bbox = np.array([440,227,300,200])
        else:
            uv = np.empty([2,8])
            bbox_stamps = [rospy.Time()]*8
            
            for i in range(0,8):
                uv[:,i] = self.cam3DToUV_(points[i])
                bbox_stamps[i] = points[i].header.stamp

            # print(uv)
            upperleft = [min(uv[0]),min(uv[1])]
            bottomright = [max(uv[0]),max(uv[1])]
            # print('upperleft[0], bottomright[0]',upperleft[0], bottomright[0])
            bbox_width = bottomright[0]-upperleft[0]
            bbox_height = bottomright[1]-upperleft[1]

            bbox = np.array([upperleft[0],upperleft[1],bbox_width,bbox_height])
        
            if self.test_limits:
                bbox = np.array([bbox[0]+self.u_lim,bbox[1]+self.v_lim,bbox[2],bbox[3]])
            
            # if self.debug:
                # bbox = np.array([440,227,300,200])
            # if bbox_stamps[0] == bbox_stamps[7]:
                # print('all bbox stamps are equal')
            bbox_stamp = bbox_stamps[7]
            # print('bbox:', bbox)
        return bbox, self.intrinsics, bbox_stamp
        
    def getCornerPoints_(self,stamp):
        """
        function that looks up tf's between the sensor's physical origin and the object's 8 corner points
        CF stands for Camera Frame
        """
        cornerPoints_CF = [None]*8
                
        for i in range(0,8):
            cornerPoints_CF[i] = PointStamped()

            parent_frame = 'robot_zed_left_sensor_link'
            if self.GT:
                child_frame = 'object_corners_{}'.format(i)
                # print('GT is on and child frame is ', child_frame)
            else:
                child_frame = 'object_corners_link_MPPI_{}'.format(i)
                # print('obj cor MPPI format')
                # object_corners_link_MPPI
            try:
                # time = self.tf_buffer.get_latest_common_time(parent_frame, child_frame)
                time = stamp
                # first attempt with rospy Time
                transform = self.tf_buffer.lookup_transform(
                        parent_frame, child_frame, time, timeout=rospy.Duration(0.5))

                # self.listener.waitForTransform(parent_frame_id, 'object_base_link', rospy.Time(0), rospy.Duration(4.0)) #tf_corner_parent.header.stamp, rospy.Duration(4.0))
                # print('latest common time of robot_zed_left_sensor_link and object_corner base', time)
                cornerPoints_CF[i].header = transform.header
                cornerPoints_CF[i].point = transform.transform.translation
                # if i==0: # or i == 2 or i==4 or i==6:
                    # pass
                    # print('cornerPoints_CF point of corner ',i,cornerPoints_CF[i].point)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException):
                # print('exception found, continue...')
                continue
        return cornerPoints_CF
    
    def cam3DToUV_(self, point_stamped):
        '''
        @param:
            - self.intrins: the camera intrinsics
            - point_stamped: PointStamped containing the 3D world coordinates of a point (box corner), expressed wrt the camera frame
        @return u,v: pixel coordinate locations mapping the 3D point to the image frame
        '''
        K = self.intrinsics

        X = point_stamped.point.x
        Y = point_stamped.point.y
        Z = point_stamped.point.z
        point_3d = np.array([X,Y,Z])

        # print('point_3d.shape',point_3d)
        # point_3d_hom = np.vstack((point_3d, 1))
        point_2d_hom = np.dot(K, point_3d.transpose())
        # convert 3D point to homogeneous coordinates
        
        # project the point to the image plane
        
        # convert homogeneous coordinates to pixel coordinates
        point_2d = point_2d_hom[:2] / point_2d_hom[2]
        
        # round pixel coordinates to nearest integer values
        pixel_coords = np.round(point_2d).astype(int)
        
        return pixel_coords
        
def get_roi(img, Bbox, crop_size, interpolation, resize_method):
    if resize_method == "crop_resize":
        roi = crop_resize(img, Bbox, crop_size, interpolation = interpolation)
        return roi
    elif resize_method == "crop_resize_by_warp_affine":
        scale, bbox_center = get_scale_and_Bbox_center(Bbox, img)
        roi = crop_resize_by_warp_affine(img, bbox_center, scale, crop_size, interpolation = interpolation)
        return roi
    elif resize_method == "crop_square_resize":
        roi = crop_square_resize(img, Bbox, crop_size, interpolation=interpolation)
        return roi
    else:
        raise NotImplementedError(f"unknown decoder type: {resize_method}")

def get_scale_and_Bbox_center(Bbox, image):
    x1 = Bbox[0]
    bw = Bbox[2]
    x2 = Bbox[0] + bw
    y1 = Bbox[1]
    bh = Bbox[3]
    y2 = Bbox[1] + bh

    bbox_center = np.array([0.5 * (x1 + x2), 0.5 * (y1 + y2)])
    if bh > bw:
        x1 = bbox_center[0] - bh/2
        x2 = bbox_center[0] + bh/2
    else:
        y1 = bbox_center[1] - bw/2
        y2 = bbox_center[1] + bw/2

    scale = max(bh, bw)
    scale = min(scale, max(image.shape[0], image.shape[1])) *1.0
    return scale, bbox_center

def crop_resize_by_warp_affine(img, center, scale, output_size, rot=0, interpolation=cv2.INTER_LINEAR):
    """
    output_size: int or (w, h)
    NOTE: if img is (h,w,1), the output will be (h,w)
    """
    if isinstance(scale, (int, float)):
        scale = (scale, scale)
    if isinstance(output_size, int):
        output_size = (output_size, output_size)
    trans = get_affine_transform(center, scale, rot, output_size)

    dst_img = cv2.warpAffine(img, trans, (int(output_size[0]), int(output_size[1])), flags=interpolation)

    return dst_img

def crop_square_resize(img, Bbox, crop_size=None, interpolation=None):
    x1 = Bbox[0]
    bw = Bbox[2]
    x2 = Bbox[0] + bw
    y1 = Bbox[1]
    bh = Bbox[3]
    y2 = Bbox[1] + bh

    bbox_center = np.array([0.5 * (x1 + x2), 0.5 * (y1 + y2)])
    if bh > bw:
        x1 = bbox_center[0] - bh/2
        x2 = bbox_center[0] + bh/2
    else:
        y1 = bbox_center[1] - bw/2
        y2 = bbox_center[1] + bw/2

    x1 = int(x1)
    y1 = int(y1)
    x2 = int(x2)
    y2 = int(y2)

    # print('max(bh, bw)',max(bh, bw))
    if img.ndim > 2:
        roi_img = np.zeros((max(bh, bw), max(bh, bw), img.shape[2]), dtype=img.dtype)
    else:
        roi_img = np.zeros((max(bh, bw), max(bh, bw)), dtype=img.dtype)
    roi_x1 = max((0-x1), 0)
    x1 = max(x1, 0)
    roi_x2 = roi_x1 + min((img.shape[1]-x1), (x2-x1))
    roi_y1 = max((0-y1), 0)
    y1 = max(y1, 0)
    roi_y2 = roi_y1 + min((img.shape[0]-y1), (y2-y1))
    x2 = min(x2, img.shape[1])
    y2 = min(y2, img.shape[0])

    roi_img[roi_y1:roi_y2, roi_x1:roi_x2] = img[y1:y2, x1:x2].copy()
    roi_img = cv2.resize(roi_img, (crop_size,crop_size), interpolation=interpolation)
    return roi_img

def crop_resize(img, Bbox, crop_size=None, interpolation=None):
    x1 = max(0, Bbox[0])
    x2 = min(img.shape[1], Bbox[0]+Bbox[2])
    y1 = max(0, Bbox[1])
    y2 = min(img.shape[0], Bbox[1]+Bbox[3])
    ####
    x1 = max(x1, 0)
    y1 = max(y1, 0)
    x2 = min(x2, img.shape[1])
    y2 = min(y2, img.shape[0])
    ####

    img = img[y1:y2, x1:x2]
    roi_img = cv2.resize(img, (crop_size, crop_size), interpolation = interpolation)
    return roi_img
