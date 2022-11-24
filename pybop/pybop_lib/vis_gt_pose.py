#!/usr/bin/env python
import os
import json
#import yaml
import argparse
import math
import cv2
import numpy as np
import kalibr_common as kc

''' syntax:
    python ~<path_to_script>Husky_scripts/bop_ros_conversion/src/scripts/pnp.py --cam <path_to_kalibr_camchain_yaml_file> --src_im <path_to_image> --tgt_dir <path_to_output_image (visualisation to confirm a correct world coordinate frame)>
'''


class Camera():
  def __init__(self, **kwargs):
    self.__dict__.update(kwargs)
  #def __getitem__(self, item):
  #  return self.Fruits[item]

class Image():
  def __init__(self, inpath, outpath, coords_2D, coords_3D, tilesize):
    self.path = inpath
    self.outpath = outpath
    self.coords_2D = coords_2D
    self.coords_3D = coords_3D
    self.tilesize = tilesize
    print(self.coords_3D)
    #self.boardlocation = boardlocation
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
def get_img(camera, image, leftright, args):
#    success, vector_rotation, vector_translation = calc_extrins(path)
#    print('Did it succeed?', success)
#    print('Rotation and translation vector', vector_rotation, vector_translation)
    img = cv2.imread(image.path)
    if leftright == "right":
        camno = 1
    else:
        camno = 0

    # select camera
    cam = camera[camno]
    # convert intrinsics to the desired numpy array format for opencv
    intrinsics = np.array([
                                 [cam.intrinsics[0], 0                , cam.intrinsics[2] ],
                                 [0,                 cam.intrinsics[1], cam.intrinsics[3] ],
                                 [0,                 0                , 1                 ]
                          ], dtype = "double")
    #print('image coords 2D: \n',(image.coords_2D))

    success, vrot_init, vtrans_init = cv2.solvePnP(image.coords_3D,image.coords_2D, intrinsics, cam.distortion_coeffs, flags=0)
    vrot, vtrans = cv2.solvePnPRefineLM(image.coords_3D,image.coords_2D, intrinsics, cam.distortion_coeffs, vrot_init, vtrans_init)
    world_frame_point2Dx, jacobian = cv2.projectPoints(np.array([(9*image.tilesize, 0.0 , 0.0  )]), vrot, vtrans, intrinsics, cam.distortion_coeffs)
    world_frame_point2Dy, jacobian = cv2.projectPoints(np.array([(0.0 , 3*image.tilesize, 0.0  )]), vrot, vtrans, intrinsics, cam.distortion_coeffs)
    world_frame_point2Dz, jacobian = cv2.projectPoints(np.array([(0.0 , 0.0 , 4*image.tilesize )]), vrot, vtrans, intrinsics, cam.distortion_coeffs)

    #print('camcoords world origin: ',world_frame_point2Dx)

    #for p in image_points_2D:
    #    cv2.circle(img, (int(p[0]), int(p[1])), 3, (0,0,255), -1)
    #if image.boardlocation == "floor":
    #    origin_idx = 5
    #elif image.boardlocation == "wall":
    #    origin_idx = 0
    if args.seq_num == "000019":
        if args.im_num == '000029':
            origin_idx = 4
        elif args.im_num == '000025':
            origin_idx = 0
        else:
            origin_idx = 5
    elif args.seq_num == '000020':
        origin_idx = 2
    else: #lif args.seq_num == "000018":
        origin_idx = 0

    # define the locations for the world coordinate axes to visualise in the image
    u_origin = int(image.coords_2D[origin_idx][0])
    v_origin = int(image.coords_2D[origin_idx][1])
    point1 = ( u_origin, v_origin )
    point2 = ( int(world_frame_point2Dx[0][0][0]), int(world_frame_point2Dx[0][0][1]))
    point3 = ( int(world_frame_point2Dy[0][0][0]), int(world_frame_point2Dy[0][0][1]))
    point4 = ( int(world_frame_point2Dz[0][0][0]), int(world_frame_point2Dz[0][0][1]))

    # BGR encoded, not RGB, so RGB for XYZ
    #print('point1: \n', point1, '\n', 'point2: \n', point2)
    #print('point3: \n', point3, '\n', 'point4: \n', point4)
    # x axis, red
    cv2.line(img, point1, point2, (0,0,255), 1)
    # y axis, green
    cv2.line(img, point1, point3, (0,255,0), 1)
    # z axis, blue
    cv2.line(img, point1, point4, (255,0,0), 1)
    #cv2.line(img, point1, point2, (255,255,255), 2)

    # find rotation matrix in 3x3 format
    rotM = cv2.Rodrigues(vrot)[0]
    #rvec_back = cv2.Rodrigues(rotM)
    # find eulerangles for easy conversion to the camera tilt
    eulerangles = rotationMatrixToEulerAngles(rotM)
    eulerdeg = np.zeros((3,1))
    eulerdeg[0] = math.degrees(eulerangles[0])-90
    eulerdeg[1] = math.degrees(eulerangles[1])
    eulerdeg[2] = math.degrees(eulerangles[2])
    cameraPosition = -np.matrix(rotM).T * np.matrix(vtrans)


    #print("Camera Matrix :")
    #print(intrinsics)
    #print("Distortion Matrix:")
    #print(distortion_coeffs)
    #print("Rotation Vector:")
    #print(vrot)
    #print("Rotation Vector back:")
    #print(rvec_back)
    print("Translation Vector:")
    print(vtrans)
    print("rotation matrix: ")
    print(rotM)
    print("euler angles: ")
    print(eulerangles)
    print("euler angles in degrees: ")
    print(eulerdeg)
    print("cam pos with respect to the world origin: ")
    print(cameraPosition)
    print('camera tilt angle is :', eulerdeg[0])

    # backproject a point far from the origin to see if it alignes nicely
    if args.seq_num == "000018":
        Xw = 9*image.tilesize
        Yw = 0*image.tilesize
        Zw = 3*image.tilesize #4*block

    if args.seq_num == "000019":
        Xw = 5*image.tilesize
        Yw =-2*image.tilesize
        Zw = 0*image.tilesize #4*block
        bp_idx = 3
    if args.seq_num == "000019" and args.im_num == '000025':
        Xw =-4*image.tilesize
        Yw = 3*image.tilesize
        Zw = 0*image.tilesize #4*block
        bp_idx = 3
    if args.seq_num == "000015":
        Xw = 4*image.tilesize
        Yw = 0*image.tilesize
        Zw = 4*image.tilesize #4*block
    if args.seq_num == "000020":
        Xw = 5*image.tilesize
        Yw = 0*image.tilesize
        Zw = -4*image.tilesize #4*block
    Xc, Yc, Zc = world3DtoCam3D(rotM, vtrans, Xw, Yw, Zw)
    print('Xc Yc Zc is: ',Xc, Yc, Zc)

    # calculate pixel coordinates of backprojected 3D point
    bpu, bpv = cam3D_to_UV(intrinsics, Xc, Yc, Zc)
    error = math.sqrt( (abs(image.coords_2D[bp_idx][0] - bpu))**2 +  (abs(image.coords_2D[bp_idx][1] - bpv))**2 )
    print('reprojection error in pixels is: ',error)
    circle_radius = 2
    circle_thickness = 1
    # bp for backprojected
    cv2.circle(img, (int(bpu), int(bpv)), circle_radius+1, (0,0,0), circle_thickness)
    # also visualise all manually annotated points to verify that they were correct
    cv2.circle(img, (int(image.coords_2D[0][0]), int(image.coords_2D[0][1])), circle_radius, (0,0,255), circle_thickness)
    cv2.circle(img, (int(image.coords_2D[1][0]), int(image.coords_2D[1][1])), circle_radius, (0,255,0), circle_thickness)
    cv2.circle(img, (int(image.coords_2D[2][0]), int(image.coords_2D[2][1])), circle_radius, (255,0,0), circle_thickness)
    cv2.circle(img, (int(image.coords_2D[3][0]), int(image.coords_2D[3][1])), circle_radius, (0,255,255), circle_thickness)
    cv2.circle(img, (int(image.coords_2D[4][0]), int(image.coords_2D[4][1])), circle_radius, (255,255,0), circle_thickness)
    cv2.circle(img, (int(image.coords_2D[5][0]), int(image.coords_2D[5][1])), circle_radius, (255,0,255), circle_thickness)
    cv2.circle(img, (int(image.coords_2D[6][0]), int(image.coords_2D[6][1])), circle_radius, (255,255,255), circle_thickness)
    #print('u and v are: ',u, v)
    #cv2.imshow("Final",img)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    # write to path for increased speed (imshow was very slow)
    cv2.imwrite(image.outpath,img)
    #return img


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="calculate camera pose relative to a known checkerboard")
    parser.add_argument('--target', dest='targetYaml', help='Calibration target configuration as yaml file') #, required=True)
    parser.add_argument('--cam', dest='chainYaml', help='Camera configuration as yaml file', required=True)
    parser.add_argument("--src_im", dest='inpath', help="image with the known coordinates.", required=True)
    parser.add_argument("--tgt_dir", dest='outpath', help="directory for saving the output image.", required=True)
    #parser.add_argument("--board_location", help="location of calibration board, floor or wall", required=True)
    parser.add_argument("--cam_location", help="selection of camera, left or right", required=True)
    parser.add_argument("--im_num", help="image number, 6 figures", required=True)
    parser.add_argument("--seq_num", help="sequence number, 6 figures", required=True)

    args = parser.parse_args()
    chainYaml = args.chainYaml
    #board_loc = args.board_location
    leftright = args.cam_location
    spacing_1 = 0.034478
    spacing_2 = spacing_1*0.3
    spacing = 1000*(spacing_1+spacing_2)

    spot_spacing_1 = 0.09000
    spot_spacing_2 = 0.11500

    block = spot_spacing_2

    x = np.zeros((7,1))
    y = np.zeros((7,1))
    print('sequence number is: ',args.seq_num)
    print('cam location is ',leftright)
    print('image number is is ',args.im_num)
    print('image file path is: ',args.inpath)


    # SEQUENCE 20220909_000015 wallmounted
    if args.seq_num == '000105':
        if leftright == 'left':
            if args.im_num == '000001':
                # seq 000015 im 000011 right cam from 20220909_sequence_imu_000015/left/000011.png
                x[0] = 1294 ; y[0] = 587                   # Bottom Left
                x[1] = 1503 ; y[1] = 556                   # 2 right, 1 up
                x[2] = 1835 ; y[2] = 390                   # 4 right, 3 up
                x[3] = 1527 ; y[3] = 455                   # 2 right, 2 up
                x[4] = 1552 ; y[4] = 346                   # 2 right, 3 up
                x[5] = 1457 ; y[5] = 216                   # 1 right, 4 up
                x[6] = 1880 ; y[6] = 255                   # 4 right, 4 up
'''
    # SEQUENCE 20220909_000018 wallmounted
    if args.seq_num == '000018':
        if leftright == 'right':
            if args.im_num == '000026':
                # seq 000018 im 000026 right cam from 20220909_sequence_imu_000018/right/000026.png
                x[0] = 407 ; y[0] = 618                   # Bottom Left
                x[1] = 450 ; y[1] = 326                   # Top left
                x[2] = 1213; y[2] = 322                   # Top right
                x[3] = 1279; y[3] = 611                   # Bottom right
                x[4] = 572 ; y[4] = 429                   # Midline, 2 blocks to the right
                x[5] = 783 ; y[5] = 427                   # Midline, 4 blocks to the right
                x[6] = 995 ; y[6] = 426                   # Midline, 6 blocks to the right


            elif args.im_num == '000030':
                # seq 000018 im 000030 right cam from 20220909_sequence_imu_000018/right/000030.png
                x[0] = 372 ; y[0] = 655                   # Bottom Left
                x[1] = 412 ; y[1] = 346                   # Top left
                x[2] = 1235; y[2] = 341                   # Top right
                x[3] = 1303; y[3] = 647                   # Bottom right
                x[4] = 546 ; y[4] = 456                   # Midline, 2 blocks to the right
                x[5] = 772 ; y[5] = 454                   # Midline, 4 blocks to the right
                x[6] = 999 ; y[6] = 452                   # Midline, 6 blocks to the right

        elif leftright == 'left':
            if args.im_num == '000037':
                # seq 000018 im 000037 left cam from 20220909_sequence_imu_000018/left/000037.png
                x[0] = 420 ; y[0] = 725                   # Bottom Left
                x[1] = 473 ; y[1] = 379                   # Top left
                x[2] = 1416; y[2] = 375                   # Top right
                x[3] = 1464; y[3] = 712                   # Bottom right
                x[4] = 624 ; y[4] = 504                   # Midline, 2 blocks to the right
                x[5] = 884 ; y[5] = 502                   # Midline, 4 blocks to the right
                x[6] = 1143; y[6] = 500                   # Midline, 6 blocks to the right

            elif args.im_num == '000026':
                # seq 000018 im 000026 left cam from 20220909_sequence_imu_000018/left/000026.png
                x[0] = 506 ; y[0] = 613                   # Bottom Left
                x[1] = 560 ; y[1] = 320                   # Top left
                x[2] = 1324; y[2] = 318                   # Top right
                x[3] = 1376; y[3] = 606                   # Bottom right
                x[4] = 680 ; y[4] = 424                   # Midline, 2 blocks to the right
                x[5] = 892 ; y[5] = 423                   # Midline, 4 blocks to the right
                x[6] = 1103; y[6] = 422                   # Midline, 6 blocks to the right

            elif args.im_num == '000017':
                # seq 000018 im 000017 left cam from 20220909_sequence_imu_000018/left/000017.png
                x[0] = 560 ; y[0] = 544                   # Bottom Left
                x[1] = 614 ; y[1] = 286                   # Top left
                x[2] = 1268; y[2] = 284                   # Top right
                x[3] = 1319; y[3] = 539                   # Bottom right
                x[4] = 715 ; y[4] = 376                   # Midline, 2 blocks to the right
                x[5] = 897 ; y[5] = 375                   # Midline, 4 blocks to the right
                x[6] = 1079; y[6] = 374                   # Midline, 6 blocks to the right

    # SEQUENCE 20220909_000019
    elif args.seq_num == '000019':
        #print('sequence number is: ',args.seq_num)
        if args.im_num == '000026':
            if leftright == 'left':
                # 000026.png left cam from ./20220909_sequence_imu_000019/left/000026.png
                x[0] = 599 ; y[0] = 634                   # Bottom Left
                x[1] = 726 ; y[1] = 536                   # Top left
                x[2] = 1207; y[2] = 537                   # Top right
                x[3] = 1342; y[3] = 634                   # Bottom right
                x[4] = 785 ; y[4] = 565                   # Midline, 2 blocks to the right
                x[5] = 931 ; y[5] = 565                   # Midline, 4 blocks to the right
                x[6] = 1077; y[6] = 565                   # Midline, 6 blocks to the right
            else:
                # 000026.png right cam from ./20220909_sequence_imu_000019/right/000026.png
                x[0] = 523 ; y[0] = 627                   # Bottom Left
                x[1] = 661 ; y[1] = 533                   # Top left
                x[2] = 1134; y[2] = 534                   # Top right
                x[3] = 1253; y[3] = 629                   # Bottom right
                x[4] = 714 ; y[4] = 561                   # Midline, 2 blocks to the right
                x[5] = 858 ; y[5] = 561                   # Midline, 4 blocks to the right
                x[6] = 1002; y[6] = 561                   # Midline, 6 blocks to the right

        elif args.im_num == '000016':
            if leftright == 'left':
            # left camera 000016.png uit 20220909_sequence_imu_000019/left/
                print('seq 19 left im 16')
                x[0] = 651 ; y[0] = 544                   # Bottom Left
                x[1] = 753 ; y[1] = 472                   # Top left
                x[2] = 1170; y[2] = 474                   # Top right
                x[3] = 1278; y[3] = 545                   # Bottom right
                x[4] = 806 ; y[4] = 494                   # Midline, 2 blocks to the right
                x[5] = 931 ; y[5] = 494                   # Midline, 4 blocks to the right
                x[6] = 1057; y[6] = 494                   # Midline, 6 blocks to the right
            else:
            # right camera 000016.png uit 20220909_sequence_imu_000019/right/
                x[0] = 580 ; y[0] = 549                   # Bottom Left
                x[1] = 694 ; y[1] = 477                   # Top left
                x[2] = 1109; y[2] = 478                   # Top right
                x[3] = 1207; y[3] = 550                   # Bottom right
                x[4] = 742 ; y[4] = 499                   # Midline, 2 blocks to the right
                x[5] = 867 ; y[5] = 499                   # Midline, 4 blocks to the right
                x[6] = 993 ; y[6] = 499                   # Midline, 6 blocks to the right

        elif args.im_num == '000029':
            if leftright == 'left':
            # left camera 000029.png uit 20220909_sequence_imu_000019/left/
                print('seq 19 left im 16')
                x[0] = 602 ; y[0] = 628                   # Bottom Left
                x[1] = 715 ; y[1] = 559                   # Top left
                x[2] = 1204; y[2] = 532                    # Top right
                x[3] = 1258; y[3] = 628                   # Bottom right
                x[4] = 932 ; y[4] = 531                   # Midline, 2 blocks to the right
                x[5] = 1094; y[5] = 628                   # Midline, 4 blocks to the right
                x[6] = 765 ; y[6] = 628                   # Midline, 6 blocks to the right
            else:
            # right camera 000029.png uit 20220909_sequence_imu_000019/right/
                x[0] = 580 ; y[0] = 549                   # Bottom Left
                x[1] = 694 ; y[1] = 477                   # Top left
                x[2] = 1109; y[2] = 478                   # Top right
                x[3] = 1207; y[3] = 550                   # Bottom right
                x[4] = 742 ; y[4] = 499                   # Midline, 2 blocks to the right
                x[5] = 867 ; y[5] = 499                   # Midline, 4 blocks to the right
                x[6] = 993 ; y[6] = 499                   # Midline, 6 blocks to the right
        elif args.im_num == '000025':
            if leftright == 'right':
            # right camera 000025.png uit 20220909_sequence_imu_000019/
                print('seq 19 right im 25')
                x[0] = 927 ; y[0] = 621                   # Bottom Left
                x[1] = 766 ; y[1] = 621                   # Top left
                x[2] = 528 ; y[2] = 620                    # Top right
                x[3] = 664 ; y[3] = 528                   # Bottom right
                x[4] = 741 ; y[4] = 503                   # Midline, 2 blocks to the right
                x[5] = 932 ; y[5] = 504                   # Midline, 4 blocks to the right
                x[6] = 1142; y[6] = 556                   # Midline, 6 blocks to the right

    #floor mounted board 2D coordinates
   # if args.seq_num == '000019':



    if args.im_num == '000017' and args.seq_num == '000020':
        x = np.zeros((12,1))
        y = np.zeros((12,1))
        # seq 000020 im 000017 left cam
        x[0] = 431 ; y[0] = 382                   # Bottom Left
        x[1] = 690 ; y[1] = 241                   # 2 right, 1 up
        x[2] = 981 ; y[2] = 241                   # 4 right, 3 up
        x[3] = 1270; y[3] = 245                   # 2 right, 2 up
        x[4] = 1553; y[4] = 250                   # 2 right, 3 up
        x[5] = 1523; y[5] = 390                   # 1 right, 4 up
        x[6] = 1655; y[6] = 392                   # 4 right, 4 up
        x[7] = 1560; y[7] = 737                   # 4 right, 4 up
        x[8] = 1348; y[8] = 632                   # 4 right, 4 up
        x[9] = 975 ; y[9] = 513                   # 4 right, 4 up
        x[10]= 598 ; y[10]= 628                   # 4 right, 4 up
        x[11]= 476 ; y[11]= 627                   # 4 right, 4 up

'''
        coords = np.array(
                      [   (x[0], y[0] ),                   # Bottom left
                          (x[1], y[1] ),                   # Top left
                          (x[2], y[2] ),                   # Top right
                          (x[3], y[3] ),                   # Bottom right
                          (x[4], y[4] ),                   # Midline, 2 blocks to the right
                          (x[5], y[5] ),                   # Midline, 4 blocks to the right
                          (x[6], y[6] ),                   # Midline, 6 blocks to the right
                          (x[7], y[7] ),                   # Midline, 6 blocks to the right
                          (x[8], y[8] ),                   # Midline, 6 blocks to the right
                          (x[9], y[9] ),                   # Midline, 6 blocks to the right
                          (x[10], y[10] ),                 # Midline, 6 blocks to the right
                          (x[11], y[11] ),                 # Midline, 6 blocks to the right
                      ], dtype="double")

    else:
        coords = np.array(
                      [   (x[0], y[0] ),                   # Bottom left
                          (x[1], y[1] ),                   # Top left
                          (x[2], y[2] ),                   # Top right
                          (x[3], y[3] ),                   # Bottom right
                          (x[4], y[4] ),                   # Midline, 2 blocks to the right
                          (x[5], y[5] ),                   # Midline, 4 blocks to the right
                          (x[6], y[6] ),                   # Midline, 6 blocks to the right
                      ], dtype="double")



    if args.seq_num == "000019": #floor mounted
        if args.im_num == '000029':
            worldcoords = np.array([
                            (-4*block,-3*block , 0.0 ),             # Bottom Left
                            (-3*block,-1*block , 0.0 ),             # Top left
                            (4*block , 0*block , 0.0 ),             # Top right
                            (4*block ,-3*block , 0.0 ),             # Bottom right
                            (0*block , 0*block , 0.0 ),             # Top center
                            (2*block ,-3*block , 0.0 ),             # mid right
                            (-2*block,-3*block , 0.0 )              # mid left
                        ], dtype="double")
        elif args.im_num == '000025' and leftright == 'right':
            worldcoords = np.array([
                            ( 0*block , 0*block , 0.0 ),             # Bottom Left
                            (-2*block , 0*block , 0.0 ),             # Top left
                            (-5*block , 0*block , 0.0 ),             # Top right
                            (-4*block , 3*block , 0.0 ),             # Bottom right
                            (-3*block , 4*block , 0.0 ),             # Top center
                            ( 0*block , 4*block , 0.0 ),             # mid right
                            ( 3*block , 2*block , 0.0 )              # mid left
                        ], dtype="double")

        else:
            worldcoords = np.array([
                            (-4*block,-2*block , 0.0 ),             # Bottom Left
                            (-3*block, 1*block , 0.0 ),             # Top left
                            (4*block , 1*block , 0.0 ),             # Top right
                            (5*block ,-2*block , 0.0 ),             # Bottom right
                            (-2*block, 0*block , 0.0 ),             # Midline, 2 blocks to the right
                            (0*block , 0*block , 0.0 ),             # Midline, 4 blocks to the right
                            (2*block , 0*block , 0.0 )              # Midline, 6 blocks to the right
                        ], dtype="double")

    elif args.seq_num == "000018": # wall mounted
        worldcoords = np.array([
                            (0.0     , 0.0 , 0.0    ),             # Bottom Left
                            (1*block , 0.0 , 3*block),             # Top left
                            (8*block , 0.0 , 3*block),             # Top right
                            (9*block , 0.0 , 0.0    ),             # Bottom right
                            (2*block , 0.0 , 2*block),             # Midline, 2 blocks to the right
                            (4*block , 0.0 , 2*block),             # Midline, 4 blocks to the right
                            (6*block , 0.0 , 2*block)              # Midline, 6 blocks to the right
                        ], dtype="double")
    elif args.seq_num == "000020": # wall mounted
        worldcoords = np.array([
                            (-4*block, 0.0 ,-1*block),             # Top left
                            (-2*block, 0.0 , 0*block),             # Top right
                            (0.0     , 0.0 , 0.0    ),             # Bottom Left
                            (2*block , 0.0 , 0*block),             # Bottom right
                            (4*block , 0.0 , 0*block),             # Midline, 2 blocks to the right
                            (4*block , 0.0 ,-1*block),             # Midline, 4 blocks to the right
                            (5*block , 0.0 ,-1*block),              # Midline, 6 blocks to the right
                            (5*block , 0.0 ,-4*block),              # Midline, 6 blocks to the right
                            (3*block , 0.0 ,-3*block),              # Midline, 6 blocks to the right
                            (0*block , 0.0 ,-2*block),              # Midline, 6 blocks to the right
                            (-3*block, 0.0 ,-3*block),              # Midline, 6 blocks to the right
                            (-4*block, 0.0 ,-3*block)              # Midline, 6 blocks to the right
                        ], dtype="double")



    cam = dict()
    im = Image(
              args.inpath,
              args.outpath,
              #2D_coords = np.zeros((6,2))
              #calib_target = "spot", #2d_coords = {{},{},{}}
              coords_2D = coords,
              #np.array(                      [   (x[0], y[0] ),                   # Top left                          (x[1], y[1] ),                   # Top right                          (x[2], y[2] ),                   # Bottom Lefr                          (x[3], y[3] ),                   # Bottom right                          (x[4], y[4] ),                   # Midline, 2 blocks to the right                          (x[5], y[5] ),                   # Midline, 4 blocks to the righ                          (x[6], y[6] ),                   # Midline, 6 blocks to the right                      ], dtype="double"),

              # x up, y to the right, z into the screen
              coords_3D = worldcoords,
              tilesize = block
              )
    #print('these image coorinates should be nonzero',im.coords_3D)
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
    get_img(cam, im, leftright, args)

