#!/usr/bin/env python
import os
import json
#import yaml
import argparse
import math
import cv2
import numpy as np
import kalibr_common as kc

class Camera():
  def __init__(self, **kwargs):
    self.__dict__.update(kwargs)
  #def __getitem__(self, item):
  #  return self.Fruits[item]

class Image():
  def __init__(self, path, coords_2D, coords_3D):
    self.path = path
    self.coords_2D = coords_2D
    self.coords_3D = coords_3D
  #  self.__dict__.update(kwargs)
  #def __getitem__(self, item):
  #  return self.Fruits[item]

def getCamParams(chainYaml,cam_no):
  camchain = kc.ConfigReader.CameraChainParameters(chainYaml)
  camConfig = camchain.getCameraParameters(cam_no)
  cam_model, intrinsics = camConfig.getIntrinsics()
  dist_model, dist_coeffs = camConfig.getDistortion()
  resolution = camConfig.getResolution()
  dist_coeffs_np = np.zeros((4,1))
  dist_coeffs_np[0] = dist_coeffs[0]; dist_coeffs_np[1] = dist_coeffs[1]; dist_coeffs_np[2] = dist_coeffs[2]; dist_coeffs_np[3] = dist_coeffs[3]
  return cam_model, intrinsics, dist_model, dist_coeffs_np, resolution


def cam3D_to_UV(intrins, Xc, Yc, Zc):
  camvec = np.array([Xc,Yc,Zc])
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
def get_img(camera, image):
#    success, vector_rotation, vector_translation = calc_extrins(path)
#    print('Did it succeed?', success)
#    print('Rotation and translation vector', vector_rotation, vector_translation)
    img = cv2.imread(image.path)

    size = img.shape #print(type(camera[0].distortion_coeffs)) #.distortion_coeffs)
    # sequence_000007_8_calib_joint.bag calibration, created on 2022-09-08
    # kalibr values for left cam
    # distortion: [-0.04844467  0.01017392  0.00105012 -0.00170879] +- [0.00088042 0.00051848 0.00028137 0.00016903]
    # projection: [1094.3380438  1103.01770299  938.3323139   538.4169745 ] +- [0.26297523 0.24147777 0.34853305 0.16415627]
    #distortion_coeffs = np.zeros((4,1))
    #distortion_coeffs[0] = -0.04844467
    #distortion_coeffs[1] = 0.01017392
    #distortion_coeffs[2] = 0.00105012
    #distortion_coeffs[3] = -0.00170879
    #focal_length_x = 1094.3380438
    #focal_length_y = 1103.0177029
    #center = np.zeros((2,1))
    #center[0] = 938.3323139 #632.02433372
    #center[1] = 538.4169745 #360.1654852

    # Values below correspond with image 000240.png from sequence_000008_calib.bag
    #image_points_2D = np.array([
    #                    (748 /cr, 32  /cr),  # Top left
    #                    (946 /cr, 34  /cr),  # Top right
    #                    (757 /cr, 234 /cr),  # Bottom Left
    #                    (943 /cr, 236 /cr),   # Bottom right
    #                    (753 /cr, 136 /cr),  # Mid left top left
    #                    (848 /cr, 137 /cr),  # Mid center top left
    #                    (945 /cr, 138 /cr)  # Mid Right top right
    #                  ], dtype="double")

    #figure_points_3D = np.array([
    #                        (0.0  , 0.0, whole),           # Top left
    #                        (whole, 0.0, whole),           # Top right
    #                        (0.0  , 0.0, 0.0  ),           # Bottom Left
    #                        (whole, 0.0, 0.0  ),           # Bottom right
    #                        (0.0  , 0.0, half ),           # Mid left top left #(whole, whole, 0.0),
    #                        (half , 0.0, half ),           # Mid center top left
    #                        (whole, 0.0, half ),           # Mid Right top left
    #                    ])
    block = 0.115
    points_3D = np.array([
                            (0.0    , 1*block  , 3*block),             # Top left
                            (0.0    , 8*block  , 3*block),             # Top right
                            (0.0    , 0.0      , 0.0    ),             # Bottom Left
                            (0.0    , 9*block  , 0.0    ),             # Bottom right
                            (0.0    , 2*block  , 2*block),             # Midline, 2 blocks to the right
                            (0.0    , 4*block  , 2*block),             # Midline, 4 blocks to the right
                            (0.0    , 6*block  , 2*block),             # Midline, 6 blocks to the right
                        ])

    #intrinsics = np.array([
    #                             [focal_length_x,  0,              center[0] ],
    #                             [0,               focal_length_y, center[1] ],
    #                             [0,               0,              1         ]
    #                         ], dtype = "double")

    # select left camera
    cam = camera[0]
    # convert intrinsics to the desired numpy array format for opencv
    intrinsics = np.array([
                                 [cam.intrinsics[0], 0                , cam.intrinsics[2] ],
                                 [0,                 cam.intrinsics[1], cam.intrinsics[3] ],
                                 [0,                 0                , 1                 ]
                          ], dtype = "double")
    print(type(image.coords_2D))

    #mage.coords3D
    #2D_coords = image.coords2D
    #cam = camera[0]
    #success, vrot, vtrans = cv2.solvePnP(figure_points_3D, image_points_2D, intrinsics, distortion_coeffs, flags=0)
    success, vrot, vtrans = cv2.solvePnP(image.coords_3D,image.coords_2D, intrinsics, cam.distortion_coeffs, flags=0)
    world_frame_point2Dx, jacobian = cv2.projectPoints(np.array([(500.0, 0.0 , 0.0  )]), vrot, vtrans, intrinsics, cam.distortion_coeffs)
    world_frame_point2Dy, jacobian = cv2.projectPoints(np.array([(0.0  , 50.0, 0.0  )]), vrot, vtrans, intrinsics, cam.distortion_coeffs)
    world_frame_point2Dz, jacobian = cv2.projectPoints(np.array([(0.0  , 0.0, 1000.0)]), vrot, vtrans, intrinsics, cam.distortion_coeffs)
    #print('camcoords world origin: ',world_frame_point2Dx)

    #for p in image_points_2D:
    #    cv2.circle(img, (int(p[0]), int(p[1])), 3, (0,0,255), -1)

    point1 = ( int(image.coords_2D[2][0]), int(image.coords_2D[2][1]))
    point2 = ( int(world_frame_point2Dx[0][0][0]), int(world_frame_point2Dx[0][0][1]))
    point3 = ( int(world_frame_point2Dy[0][0][0]), int(world_frame_point2Dy[0][0][1]))
    point4 = ( int(world_frame_point2Dz[0][0][0]), int(world_frame_point2Dz[0][0][1]))
    #point1 = ( int(image_points_2D[2][0]), int(image_points_2D[2][1]))
    #point2 = ( int(world_frame_point2Dx[0][0][0]), int(world_frame_point2Dx[0][0][1]))
    #point3 = ( int(world_frame_point2Dy[0][0][0]), int(world_frame_point2Dy[0][0][1]))
    #point4 = ( int(world_frame_point2Dz[0][0][0]), int(world_frame_point2Dz[0][0][1]))
    #print(point1)
    #print(point2)

    # wp0 = int(
    # BGR, not RGB
    cv2.line(img, point1, point2, (0,0,255), 1)
    cv2.line(img, point1, point3, (0,255,0), 1)
    cv2.line(img, point1, point4, (255,0,0), 1)
    #cv2.line(img, point1, point2, (255,255,255), 2)

    #print(success, '\n',vrot, '\n', vtrans, '\n') #, jacobian)


    rotM = cv2.Rodrigues(vrot)[0]
    #rvec_back = cv2.Rodrigues(rotM)
    eulerangles = rotationMatrixToEulerAngles(rotM)
    cameraPosition = -np.matrix(rotM).T * np.matrix(vtrans)


    #print("Camera Matrix :")
    #print(intrinsics)
    #print("Distortion Matrix:")
    #print(distortion_coeffs)
    print("Rotation Vector:")
    print(vrot)
    #print("Rotation Vector back:")
    #print(rvec_back)
    print("Translation Vector:")
    print(vtrans)
    print("rotation matrix: ")
    print(rotM)
    print("euler angles: ")
    print(eulerangles)
    print("cam pos: ")
    print(cameraPosition)

    Xw = 0
    Yw = 0
    Zw = 0

    Xc, Yc, Zc = world3DtoCam3D(rotM, vtrans, Xw, Yw, Zw)
    print('Xc Yc Zc is: ',Xc, Yc, Zc)
    u, v = cam3D_to_UV(intrinsics, Xc, Yc, Zc)
    #print(type(img))
    cv2.circle(img, (int(u), int(v)), 3, (255,255,255), 1)
    #print('u and v are: ',u, v)
    cv2.imshow("Final",img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    #return img


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="calculate camera pose relative to a known checkerboard")
    parser.add_argument('--target', dest='targetYaml', help='Calibration target configuration as yaml file') #, required=True)
    parser.add_argument('--cam', dest='chainYaml', help='Camera configuration as yaml file', required=True)
    parser.add_argument("--src_im", help="image with the known coordinates.")
    args = parser.parse_args()
    #src_im = args.src_im
    #tgt_dir = args.tgt_dir
    chainYaml = args.chainYaml

    spacing_1 = 0.034478
    spacing_2 = spacing_1*0.3
    spacing = 1000*(spacing_1+spacing_2)

    spot_spacing_1 = 0.09000
    spot_spacing_2 = 0.11500
    spot_white_border = (spot_spacing_2 - spot_spacing_1) * 0.5
    spot_block_pair = spot_spacing_2 + spot_spacing_1 + 2*spot_white_border
    block = spot_spacing_2

    x = np.zeros((7,1))
    y = np.zeros((7,1))

    # 000017.png uit 20220909_sequence_imu_000018/left/
    x[0] = 614 ; y[0] = 286                   # Top left
    x[1] = 1176; y[1] = 284                   # Top right
    x[2] = 560 ; y[2] = 544                   # Bottom Left
    x[3] = 1319; y[3] = 539                   # Bottom right
    x[4] = 715 ; y[4] = 376                   # Midline, 2 blocks to the right
    x[5] = 897 ; y[5] = 375                   # Midline, 4 blocks to the right
    x[6] = 1079; y[6] = 374                   # Midline, 6 blocks to the right
    coords = np.array(
                      [   (x[0], y[0] ),                   # Top left
                          (x[1], y[1] ),                   # Top right
                          (x[2], y[2] ),                   # Bottom Left
                          (x[3], y[3] ),                   # Bottom right
                          (x[4], y[4] ),                   # Midline, 2 blocks to the right
                          (x[5], y[5] ),                   # Midline, 4 blocks to the right
                          (x[6], y[6] ),                   # Midline, 6 blocks to the right
                      ], dtype="double")
    cam = dict()
    im = Image(
              args.src_im,
              #2D_coords = np.zeros((6,2))
              #calib_target = "spot", #2d_coords = {{},{},{}}
              coords_2D = np.array(
                      [   (x[0], y[0] ),                   # Top left
                          (x[1], y[1] ),                   # Top right
                          (x[2], y[2] ),                   # Bottom Left
                          (x[3], y[3] ),                   # Bottom right
                          (x[4], y[4] ),                   # Midline, 2 blocks to the right
                          (x[5], y[5] ),                   # Midline, 4 blocks to the right
                          (x[6], y[6] ),                   # Midline, 6 blocks to the right
                      ], dtype="double"),

              # x up, y to the right, z into the screen
              coords_3D = np.array([
                            (0.0    , 1*block  , 3*block),             # Top left
                            (0.0    , 8*block  , 3*block),             # Top right
                            (0.0    , 0.0      , 0.0    ),             # Bottom Left
                            (0.0    , 9*block  , 0.0    ),             # Bottom right
                            (0.0    , 2*block  , 2*block),             # Midline, 2 blocks to the right
                            (0.0    , 4*block  , 2*block),             # Midline, 4 blocks to the right
                            (0.0    , 6*block  , 2*block)              # Midline, 6 blocks to the right
                            #(3*block, 1*block      , 0.0),                 # Top left
                            #(3*block, 8*block      , 0.0),                 # Top right
                            #(0.0    , 0.0          , 0.0),                 # Bottom Left
                            #(0.0    , 9*block      , 0.0),                 # Bottom right
                            #(2*block, 2*block      , 0.0),           # Midline, 2 blocks to the right
                            #(2*block, 4*block      , 0.0),           # Midline, 4 blocks to the right
                            #(2*block, 6*block      , 0.0),           # Midline, 6 blocks to the right
                        ], dtype="double")
              )
    print(im.coords_3D)
    for i in range(0,2):
        #cam_model_l, intrinsics_l, dist_model_l, dist_coeffs_l, resolution_l = getCamParams(chainYaml, 0)
        cam_model, intrins, dist_mod, dist_coef, reso = getCamParams(chainYaml, i)
        cam[i] = Camera(
                        model = cam_model,
                        intrinsics = intrins,
                        distortion_model = dist_mod,
                        distortion_coeffs = dist_coef,
                        resolution = reso
                        )
    #print(intrinsics[0])
    #print(type(cam[0].distortion_coeffs))
    #print(cam[1].intrinsics[0])
    get_img(cam, im)

