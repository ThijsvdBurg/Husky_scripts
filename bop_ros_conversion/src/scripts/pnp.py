#!/usr/bin/env python
import os
import json
#import yaml
import argparse
import math
import cv2
import numpy as np

#parser = argparse.ArgumentParser(description="Rewrite header stamps and put them in new bag file")
#parser.add_argument("--src_im", help="image with the known coordinates.")
#parser.add_argument("--tgt_dir", help="Output directory.")

#args = parser.parse_args()
#src_im = args.src_im
#tgt_dir = args.tgt_dir

def cam3D_to_UV(intrins, Xc, Yc, Zc):
  camvec = np.array([Xc,Yc,Zc])
  print(camvec)
  uvw = np.dot( intrins, camvec )
  u = uvw[0] / uvw[2]
  v = uvw[1] / uvw[2]
  print('u: ',u)
  print('v: ',v)
  return u, v

def world3DtoCam3D_2(R, t, Xw, Yw, Zw):
  print(R, type(R))
  print(t, type(t))
  Rt = np.hstack([R,t]) #np.array([R,t])
  print(Rt, type(Rt), Rt.shape)

def world3DtoCam3D(R, t, Xw, Yw, Zw):
  cam3d = np.dot( np.hstack([R,t]), np.vstack([Xw,Yw,Zw,1]) )
  Xc = cam3d[0]
  Yc = cam3d[1]
  Zc = cam3d[2]
  return Xc, Yc, Zc



def isRotationMatrix(R):
  Rt = np.transpose(R)
  shouldBeI = np.dot(Rt,R)
  I = np.identity(3, dtype= R.dtype)
  n = np.linalg.norm(I - shouldBeI)
  return n < 1e-6


def rotationMatrixToEulerAngles(R):
  assert(isRotationMatrix(R))
  sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
  singular = sy < 1e-6
  if  not singular :
    x = math.atan2(R[2,1] , R[2,2])
    y = math.atan2(-R[2,0], sy)
    z = math.atan2(R[1,0], R[0,0])
  else :
    x = math.atan2(-R[1,2], R[1,1])
    y = math.atan2(-R[2,0], sy)
    z = 0
  return np.array([x, y, z])

 #def calc_extrins(img_path):
def get_img(ubuntu):
    if ubuntu:
        img_path =  '/home/pmvanderburg/noetic-husky/kalibr_workspace/bagfiles/20220908/sequence_000008_images/000240.png'
    else:
        img_path = '/Users/pmvanderburg/Downloads/000240.png'
#    success, vector_rotation, vector_translation = calc_extrins(path)
#    print('Did it succeed?', success)
#    print('Rotation and translation vector', vector_rotation, vector_translation)
    img = cv2.imread(img_path)

    size = img.shape
    cr = 1280/1280
    spacing_1 = 0.034478
    spacing_2 = spacing_1*0.3
    spacing = 1000*(spacing_1+spacing_2)
    # sequence_000007_8_calib_joint.bag calibration, created on 2022-09-08
    # kalibr values for left cam
    # distortion: [-0.04844467  0.01017392  0.00105012 -0.00170879] +- [0.00088042 0.00051848 0.00028137 0.00016903]
    # projection: [1094.3380438  1103.01770299  938.3323139   538.4169745 ] +- [0.26297523 0.24147777 0.34853305 0.16415627]
    distortion_coeffs = np.zeros((4,1))
    distortion_coeffs[0] = -0.04844467
    distortion_coeffs[1] = 0.01017392
    distortion_coeffs[2] = 0.00105012
    distortion_coeffs[3] = -0.00170879
    focal_length_x = 1094.3380438
    focal_length_y = 1103.0177029
    center = np.zeros((2,1)) # (size[1]/2, size[0]/2)
    center[0] = 938.3323139 #632.02433372
    center[1] = 538.4169745 #360.1654852

    # Values below correspond with image 000240.png from sequence_000008_calib.bag
    image_points_2D = np.array([
                        (748 /cr, 32  /cr),  # Top left
                        (946 /cr, 34  /cr),  # Top right
                        (757 /cr, 234 /cr),  # Bottom Left
                        (943 /cr, 236 /cr),   # Bottom right
                        (753 /cr, 136 /cr),  # Mid left top left
                        (848 /cr, 137 /cr),  # Mid center top left
                        (945 /cr, 138 /cr)  # Mid Right top right
                      ], dtype="double")
    whole= 6*spacing
    half = 3*spacing

    figure_points_3D = np.array([
                            (0.0  , 0.0, whole),           # Top left
                            (whole, 0.0, whole),           # Top right
                            (0.0  , 0.0, 0.0  ),           # Bottom Left
                            (whole, 0.0, 0.0  ),           # Bottom right
                            (0.0  , 0.0, half ),           # Mid left top left #(whole, whole, 0.0),
                            (half , 0.0, half ),           # Mid center top left
                            (whole, 0.0, half ),           # Mid Right top left
                        ])
    figure_points_3D_2 = np.array([
                            (whole, 0.0      , 0.0),                 # Top left
                            (whole, whole, 0.0),           # Top right
                            (0.0      , 0.0      , 0.0),                 # Bottom Left
                            (0.0      , whole, 0.0),                 # Bottom right
                            (half, 0.0      , 0.0),           # Mid left top left #(whole, whole, 0.0),
                            (half, half, 0.0),           # Mid center top left
                            (half, whole, 0.0),           # Mid Right top left
                        ])

    intrinsics = np.array([
                                 [focal_length_x,  0,              center[0] ],
                                 [0,               focal_length_y, center[1] ],
                                 [0,               0,              1         ]
                             ], dtype = "double")
    success, vector_rotation, vector_translation = cv2.solvePnP(figure_points_3D, image_points_2D, intrinsics, distortion_coeffs, flags=0)
    world_frame_point2Dx, jacobian = cv2.projectPoints(np.array([(500.0, 0.0, 0.0)]), vector_rotation, vector_translation, intrinsics, distortion_coeffs)
    world_frame_point2Dy, jacobian = cv2.projectPoints(np.array([(0.0, 50.0, 0.0)]), vector_rotation, vector_translation, intrinsics, distortion_coeffs)
    world_frame_point2Dz, jacobian = cv2.projectPoints(np.array([(0.0, 0.0, 1000.0)]), vector_rotation, vector_translation, intrinsics, distortion_coeffs)
    print('camcoords world origin: ',world_frame_point2Dx)

    #for p in image_points_2D:
    #    cv2.circle(img, (int(p[0]), int(p[1])), 3, (0,0,255), -1)

    point1 = ( int(image_points_2D[2][0]), int(image_points_2D[2][1]))
    point2 = ( int(world_frame_point2Dx[0][0][0]), int(world_frame_point2Dx[0][0][1]))
    point3 = ( int(world_frame_point2Dy[0][0][0]), int(world_frame_point2Dy[0][0][1]))
    point4 = ( int(world_frame_point2Dz[0][0][0]), int(world_frame_point2Dz[0][0][1]))
    #print(point1)
    #print(point2)

    # wp0 = int(
<<<<<<< HEAD
    # BGR, not RGB
    cv2.line(img, point1, point2, (0,0,255), 1)
=======
    cv2.line(img, point1, point2, (0,0,255), 3)
>>>>>>> 92d7682c4513910888f95a85d4a6e09760543f6c
    cv2.line(img, point1, point3, (0,255,0), 1)
    cv2.line(img, point1, point4, (255,0,0), 1)
    #cv2.line(img, point1, point2, (255,255,255), 2)

    #print(success, '\n',vector_rotation, '\n', vector_translation, '\n') #, jacobian)


    rotM = cv2.Rodrigues(vector_rotation)[0]
    #rvec_back = cv2.Rodrigues(rotM)
    eulerangles = rotationMatrixToEulerAngles(rotM)
    cameraPosition = -np.matrix(rotM).T * np.matrix(vector_translation)


    #print("Camera Matrix :")
    #print(intrinsics)
    #print("Distortion Matrix:")
    #print(distortion_coeffs)
    print("Rotation Vector:")
    print(vector_rotation)
    #print("Rotation Vector back:")
    #print(rvec_back)
    print("Translation Vector:")
    print(vector_translation)
    print("rotation matrix: ")
    print(rotM)
    print("euler angles: ")
    print(eulerangles)
    print("cam pos: ")
    print(cameraPosition)

    Xw = 0
    Yw = 0
    Zw = 0

    Xc, Yc, Zc = world3DtoCam3D(rotM, vector_translation, Xw, Yw, Zw)
    print('Xc Yc Zc is: ',Xc, Yc, Zc)
    u, v = cam3D_to_UV(intrinsics, Xc, Yc, Zc)
    #print(type(img))
    cv2.circle(img, (int(u), int(v)), 3, (255,255,255), 1)
    #print('u and v are: ',u, v)
    #cv2.imshow("Final",img)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    return img


#if __name__ == '__main__':
#    main()



