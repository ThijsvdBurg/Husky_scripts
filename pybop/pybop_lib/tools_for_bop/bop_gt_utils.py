

def gt_list(tf_stamp, rotation, translation, obj_id):
    scene_gt = [{
                'header_stamp': str(tf_stamp),
                'cam_R_m2c': rotation,
                'cam_t_m2c': translation,
                'obj_id': int(obj_id)
                 }]
    return scene_gt