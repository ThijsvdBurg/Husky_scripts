import os
# import numpy as np


def write_csv_runtime_analysis(evaluation_result_path, filename, bbox, roi, im, nn, mask, pnptot, corrs, mappix, solvepnp, total):
    """
    The function write_csv() takes the following arguments:
    
    """
    filename = os.path.join(evaluation_result_path, filename + '.csv')
    f = open(filename, "w")
    # f.write("scene_id,im_id,obj_id,score,R,t,time\n")
    f.write("meanGetBBox,meanBBoxtoROI,meanPreProcIM,meanNN,meanPostProcMasks,meanPnPTotal,mean2d3dcorrs,meanMapPixels,meanSolvePnPRANSAC,time\n")
    f.write(str(bbox))
    f.write(",")
    f.write(str(roi))
    f.write(",")
    f.write(str(im))
    f.write(",")
    f.write(str(nn))
    f.write(",")
    f.write(str(mask))
    f.write(",")
    f.write(str(pnptot))
    f.write(",")
    f.write(str(corrs))
    f.write(",")
    f.write(str(mappix))
    f.write(",")
    f.write(str(solvepnp))
    f.write(",")
    f.write(str(total))

      
      # for scene_id, img_id, r, t, score in zip(scene_id_, img_id_, r_, t_, scores):
      #     if score == -1:
      #         continue
      #     r11 = r[0][0]
      #     r12 = r[0][1]
      #     r13 = r[0][2]

      #     r21 = r[1][0]
      #     r22 = r[1][1]
      #     r23 = r[1][2]

      #     r31 = r[2][0]
      #     r32 = r[2][1]
      #     r33 = r[2][2]
      #     f.write(str(scene_id))
      #     f.write(",")
      #     f.write(str(img_id))
      #     f.write(",")
      #     f.write(str(obj_id))
      #     f.write(",")
      #     f.write(str(score)) # score
      #     f.write(",")
      #     # R
      #     f.write(str(r11))
      #     f.write(" ")
      #     f.write(str(r12))
      #     f.write(" ")
      #     f.write(str(r13))
      #     f.write(" ")
      #     f.write(str(r21))
      #     f.write(" ")
      #     f.write(str(r22))
      #     f.write(" ")
      #     f.write(str(r23))
      #     f.write(" ")
      #     f.write(str(r31))
      #     f.write(" ")
      #     f.write(str(r32))
      #     f.write(" ")
      #     f.write(str(r33))
      #     f.write(",")
      #     #t
      #     f.write(str(t[0][0]))
      #     f.write(" ")
      #     f.write(str(t[1][0]))
      #     f.write(" ")
      #     f.write(str(t[2][0]))
      #     f.write(",")
      #     #time
      #     f.write("-1\n")
    f.close()

def write_csv_list(evaluation_result_path, filename, listbbox, listroi, listim, listnn, listmask, listpnptot, listcorrs, listmappix, listsolvepnp, listtotal):
    """
    The function write_csv() takes the following arguments:

    tensor_gpu: the tensor to be visualised
    num_ch: the number of images in the tensor
    batch_id: the index of the batch to save consecutive images
    eval_output_path: path in which the resulting visualisations will be saved
    
    """
    filename = os.path.join(evaluation_result_path, filename + '_lists.csv')
    f = open(filename, "w")
    f.write("listbbox, listroi, listim, listnn, listmask, listpnptot, listcorrs, listmappix, listsolvepnp, listtotal, success\n")
    for bbox, roi, im, nn, mask, pnptot, corrs, mappix, solvepnp, total in zip(listbbox, listroi, listim, listnn, listmask, listpnptot, listcorrs, listmappix, listsolvepnp, listtotal):
      f.write(str(bbox))
      f.write(",")
      f.write(str(roi))
      f.write(",")
      f.write(str(im))
      f.write(",")
      f.write(str(nn))
      f.write(",")
      f.write(str(mask))
      f.write(",")
      f.write(str(pnptot))
      f.write(",")
      f.write(str(corrs))
      f.write(",")
      f.write(str(mappix))
      f.write(",")
      f.write(str(solvepnp))
      f.write(",")
      f.write(str(total))
      f.write(",")
      if corrs==999 or mappix==999 or solvepnp==999:
        f.write("0\n")
      else:
        f.write("1\n")

    f.close()

def write_csv_pose_analysis_ZP(evaluation_result_path, filename, obj_id, pnp_init, pnp_iters, scene_id_, img_ids_, gt_stamps_, pred_stamps_, img_stamps_, r_, t_, scores_, azi_, alti_, bbox_=None, bbox_stamps_=None, runtime=None):
    filename = os.path.join(evaluation_result_path, filename + '.csv')
    f = open(filename, "w")
    f.write("scene_id,obj_id,pnp_init,pnpiters,im_id,gt_stamp,pred_stamp,img_stamp,score,R,t,azimuth,altitude,bbox,bbox_stamp,time,\n")


    for scene_id, img_id, gt_stamp, pred_stamp, image_stamp, r, t, score, azimuth, altitude, bbox, bbox_stamp in zip(scene_id_, img_ids_, gt_stamps_, pred_stamps_, img_stamps_, r_, t_, scores_, azi_, alti_, bbox_, bbox_stamps_):
        if score == -1:
            continue
        r11 = r[0][0]
        r12 = r[0][1]
        r13 = r[0][2]

        r21 = r[1][0]
        r22 = r[1][1]
        r23 = r[1][2]

        r31 = r[2][0]
        r32 = r[2][1]
        r33 = r[2][2]

        f.write(str(scene_id))
        f.write(",")
        f.write(str(obj_id))
        f.write(",")
        f.write(str(pnp_init))
        f.write(",")
        f.write(str(pnp_iters))
        f.write(",")

        f.write(str(img_id))
        f.write(",")
        f.write(str(gt_stamp))
        f.write(",")
        f.write(str(pred_stamp))
        f.write(",")
        f.write(str(image_stamp))
        f.write(",")
        f.write(str(score)) # score
        f.write(",")
        # R
        f.write(str(r11))
        f.write(" ")
        f.write(str(r12))
        f.write(" ")
        f.write(str(r13))
        f.write(" ")
        f.write(str(r21))
        f.write(" ")
        f.write(str(r22))
        f.write(" ")
        f.write(str(r23))
        f.write(" ")
        f.write(str(r31))
        f.write(" ")
        f.write(str(r32))
        f.write(" ")
        f.write(str(r33))
        f.write(",")
        #t
        f.write(str(t[0][0]))
        f.write(" ")
        f.write(str(t[1][0]))
        f.write(" ")
        f.write(str(t[2][0]))
        f.write(",")
        # bbox, azimuth, altitude
        f.write(str(azimuth))
        f.write(",")
        f.write(str(altitude))
        f.write(",")
        if bbox_  is not None:
          f.write(str(bbox[0]))
          f.write(" ")
          f.write(str(bbox[1]))
          f.write(" ")
          f.write(str(bbox[2]))
          f.write(" ")
          f.write(str(bbox[3]))
          f.write(",")
          f.write(str(bbox_stamp))
          f.write(",")
  
        run2D = runtime[0]
        runPnP = runtime[1]
        runTotal = runtime[2]
        f.write(str(run2D))
        f.write(" ")
        f.write(str(runPnP))
        f.write(" ")
        f.write(str(runTotal))
        #time
        f.write(",\n")
    f.close()

def write_csv_pose_analysis_MPPI(evaluation_result_path, filename, obj_id, scene_id_, img_id_, gt_stamps_, pred_stamps_, r_, t_, scores_):
    filename = os.path.join(evaluation_result_path, filename + '.csv')
    f = open(filename, "w")
    f.write("scene_id,obj_id,im_id,gt_stamp,pred_stamp,score,R,t,time,,,,,,\n")

    for scene_id, img_id, gt_stamp, pred_stamp, r, t, score in zip(scene_id_, img_id_, gt_stamps_, pred_stamps_, r_, t_, scores_):
        if score == -1:
            continue
        r11 = r[0][0]
        r12 = r[0][1]
        r13 = r[0][2]

        r21 = r[1][0]
        r22 = r[1][1]
        r23 = r[1][2]

        r31 = r[2][0]
        r32 = r[2][1]
        r33 = r[2][2]

        f.write(str(scene_id))
        f.write(",")
        f.write(str(obj_id))
        f.write(",")
        f.write(str(img_id))
        f.write(",")
        f.write(str(gt_stamp))
        f.write(",")
        f.write(str(pred_stamp))
        f.write(",")
        f.write(str(score)) # score
        f.write(",")
        # R
        f.write(str(r11))
        f.write(" ")
        f.write(str(r12))
        f.write(" ")
        f.write(str(r13))
        f.write(" ")
        f.write(str(r21))
        f.write(" ")
        f.write(str(r22))
        f.write(" ")
        f.write(str(r23))
        f.write(" ")
        f.write(str(r31))
        f.write(" ")
        f.write(str(r32))
        f.write(" ")
        f.write(str(r33))
        f.write(",")
        #t
        f.write(str(t[0][0]))
        f.write(" ")
        f.write(str(t[1][0]))
        f.write(" ")
        f.write(str(t[2][0]))
        f.write(",")
        #time
        f.write("-1")
        f.write(",,,,,,\n")
    f.close()

