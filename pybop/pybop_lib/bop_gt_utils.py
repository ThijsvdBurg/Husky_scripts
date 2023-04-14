

def gt_list(stamp, rotation, translation, obj_id):
    scene_gt = [{
                'header_stamp': str(stamp),
                'cam_R_m2c': rotation,
                'cam_t_m2c': translation,
                'obj_id': int(obj_id)
                 }]
    return scene_gt

def bop_list(scene_nr, obj_id, image_id):
    bop_entry = [{'im_id': image_id, 'inst_count': 1, 'obj_id': obj_id, 'scene_id': scene_nr}]
    return bop_entry