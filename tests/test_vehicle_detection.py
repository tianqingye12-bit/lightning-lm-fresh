from pathlib import Path
import copy
import json
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
import scipy.sparse.csgraph
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import clean_map_vehicles as vr
from vehicle_detection import Detections, DetectionCache, infer, associate, detection_options
from configure_detection_cleanup import profile
from seed_local_cleanup import complete
from vote_stage_diagnostics import rejection_stages
from clean_map_instances import removal_votes
from test_alignment_round13 import fixture, config


class Tensor:
    def __init__(self, value):self.value=np.asarray(value)
    def cpu(self):return self
    def numpy(self):return self.value


class DetectionResult:
    def __init__(self, boxes, confidence=None, classes=None):
        self.boxes=SimpleNamespace(xyxy=Tensor(np.asarray(boxes).reshape(-1,4)),
            conf=Tensor(confidence if confidence is not None else [.9]*len(boxes)),
            cls=Tensor(classes if classes is not None else [2]*len(boxes)))
    @property
    def masks(self):raise AssertionError('Detection path must never read segmentation masks')


class DetectionTests(unittest.TestCase):
    def cfg(self):
        return dict(yolo_conf=.25,vehicle_classes=[2],min_views=3,vehicle_ratio=.5,
                    detection_filter=dict(min_core_pixels=3,min_component_points=3))

    def cloud(self):
        # Three connected car pixels, a disconnected point at same depth,
        # three pixels on a far wall, and a protected road pixel.
        cam=np.array([[0,0,5],[.1,0,5],[.2,0,5],[2,0,5],
                      [0,0,10],[.1,0,10],[.2,0,10],[.1,.1,5.]])
        u=np.array([4,5,6,8,4,5,6,5]);v=np.array([4,4,4,4,5,5,5,6])
        return cam,u,v,np.ones(8,bool),np.array([0,0,0,0,0,0,0,1],bool)

    def test_inference_reads_only_boxes_and_filters_class_confidence(self):
        calls=[]
        def model(image,**kwargs):
            calls.append(kwargs)
            return [DetectionResult([[0,0,10,10]]*3,[.9,.1,.9],[2,2,0])]
        detection=infer(model,np.zeros((10,10,3)),self.cfg(),'cpu')
        self.assertEqual(len(detection.boxes),1)
        self.assertNotIn('retina_masks',calls[0])
        self.assertEqual(detection.classes.tolist(),[2])

    def test_depth_and_connectivity_protect_wall_disconnected_points_and_floor(self):
        hit,bg,region,stats=associate(Detections([[0,0,10,10]],[.9],[2]),
                                     *self.cloud(),(10,10),self.cfg())
        self.assertEqual(hit.tolist(),[True,True,True,False,False,False,False,False])
        self.assertFalse(bg.any())
        self.assertTrue(region.all())  # full box overlay is not a deletion mask
        self.assertEqual(stats['accepted_components'],1)

    def test_duplicate_map_points_cannot_fake_distinct_pixel_support(self):
        cam,u,v,visible,floor=self.cloud();u[:3]=4
        hit,_,_,stats=associate(Detections([[0,0,10,10]],[.9],[2]),cam[:3],u[:3],v[:3],visible[:3],floor[:3],(10,10),self.cfg())
        self.assertFalse(hit.any());self.assertEqual(stats['skipped_core'],1)

    def test_missing_boxes_and_outside_boxes_abstain(self):
        for boxes in ([],[[20,20,30,30]]):
            hit,bg,_,_=associate(Detections(np.asarray(boxes).reshape(-1,4),[.9]*len(boxes),[2]*len(boxes)),
                                 *self.cloud(),(10,10),self.cfg())
            self.assertFalse(hit.any());self.assertFalse(bg.any())

    def test_candidate_limit_and_extent_limit_abstain(self):
        for limits,key in ((dict(max_candidates=2),'skipped_candidates'),
                           (dict(max_extent_cam_m=[.05,1,1]),'skipped_extent')):
            cfg=self.cfg();cfg['detection_filter'].update(limits)
            hit,_,_,stats=associate(Detections([[0,0,10,10]],[.9],[2]),*self.cloud(),(10,10),cfg)
            self.assertFalse(hit.any());self.assertEqual(stats[key],1)

    def test_overlapping_boxes_cast_only_one_point_vote_per_image(self):
        hit,_,_,_=associate(Detections([[0,0,10,10]]*2,[.9,.9],[2,2]),*self.cloud(),(10,10),self.cfg())
        self.assertEqual(hit.dtype,np.dtype(bool));self.assertEqual(hit.sum(),3)

    def test_each_box_has_its_own_depth_anchor(self):
        cam,u,v,visible,floor=self.cloud();u[4:7]=[14,15,16]
        hit,_,_,stats=associate(Detections([[0,0,10,10],[10,0,20,10]],[.9,.9],[2,2]),
                                cam,u,v,visible,floor,(10,20),self.cfg())
        self.assertEqual(hit.tolist(),[True,True,True,False,True,True,True,False])
        self.assertEqual(stats['accepted_components'],2)

    def test_cache_keeps_raw_boxes_not_masks_and_rejects_corruption(self):
        with tempfile.TemporaryDirectory() as tmp:
            cache=DetectionCache(tmp,dict(width=10,height=10,task='detect'))
            key=cache.key(np.zeros((10,10,3),np.uint8))
            raw=Detections([[1,2,6,8]],[.8],[2]);cache.put(key,raw,1)
            loaded,count=cache.get(key,(10,10))
            np.testing.assert_array_equal(loaded.boxes,raw.boxes);self.assertEqual(count,1)
            self.assertIsNone(cache.get(key,(11,10)))
            (Path(tmp)/key[:2]/(key+'.npz')).write_bytes(b'invalid')
            self.assertIsNone(cache.get(key,(10,10)))
            self.assertEqual(cache.corrupt,2)

    def test_no_detection_is_not_a_negative_vote(self):
        cfg=self.cfg()
        _,remove=removal_votes(np.array([100,100]),np.array([3,2]),np.zeros(2,int),cfg)
        self.assertEqual(remove.tolist(),[True,False])  # still needs three positive observations

    def test_detection_profile_preserves_sensors_and_uses_explicit_background_only(self):
        cfg=config();before=copy.deepcopy(cfg);new=profile(cfg,'yolo26n.pt')
        self.assertEqual(cfg,before)
        for key in ('K','T_cam_lidar','time_offset','yolo_conf','vehicle_classes','max_range'):
            self.assertEqual(new[key],cfg[key])
        self.assertFalse(new['legacy_map_cleanup']);self.assertFalse(new['seed_local_cleanup']['visible_negative_veto'])
        self.assertEqual(new['vehicle_perception'],'detect')

    def test_diagnostic_rejections_partition_positive_voxels(self):
        cfg=dict(min_views=3,vehicle_ratio=.5)
        visible=np.array([2,3,4,3]);positive=np.array([1,1,3,3]);floor=np.array([0,0,1,0],bool)
        result=rejection_stages(visible,positive,np.zeros(4,int),floor,np.array([0,0,0,1],bool),cfg,True)
        self.assertEqual(result['below_min_views'],1);self.assertEqual(result['below_vote_ratio'],1)
        self.assertEqual(result['floor_protection'],1);self.assertEqual(result['seed_voxels'],1)
        self.assertEqual(sum(value for key,value in result.items() if key!='positive_voxels'),4)

    def test_visible_without_box_does_not_block_detection_local_completion(self):
        cfg=profile(config(),'detector.pt');cfg['seed_local_cleanup'].update(min_seeds=1)
        result,_=complete(np.array([[0,0,1],[.1,0,1]]),np.array([True,False]),
                          np.array([3,100]),np.array([3,0]),np.zeros(2,int),np.zeros(2,bool),cfg)
        self.assertTrue(result.all())

    def test_bad_boxes_and_invalid_geometry_rejected(self):
        for boxes,confidence,classes in (([[1,1,0,2]],[.9],[2]), ([[0,0,2,2]],[float('nan')],[2]),
                                         ([[0,0,2,2]],[.9],[2.5])):
            with self.assertRaises(ValueError):Detections(boxes,confidence,classes)
        with self.assertRaises(ValueError):detection_options({'detection_filter':{'link_m':2}})

    def test_real_map_main_uses_detection_model_without_masks(self):
        with tempfile.TemporaryDirectory() as tmp:
            source=Path(tmp)/'map';fixture(source,keyframes=[[0,1,0,0,0,0,0,0,1]])
            dtype=np.dtype([(k,'<f4') for k in ('x','y','z','intensity')])
            points=[((u-16)*.2,(v-16)*.2,2.,7.) for v in (12,13,14) for u in (12,13,14)]
            points += [(0,0,8,9.)]  # wall behind box, beyond car depth band
            cloud=np.array(points,dtype=dtype)
            header=['FIELDS x y z intensity','SIZE 4 4 4 4','TYPE F F F F','COUNT 1 1 1 1','WIDTH 10','HEIGHT 1','POINTS 10','DATA binary']
            vr.write_pcd(source/'global.pcd',header,cloud)
            original=(source/'global.pcd').read_bytes()
            cfg=profile(config(),'detector.pt');cfg['seed_local_cleanup']['enabled']=False
            cfg['detection_filter'].update(min_core_pixels=3,min_component_points=3)
            path=Path(tmp)/'config.yaml';path.write_text(yaml.safe_dump(cfg))
            class Model:
                task='detect'
                def __call__(self,*args,**kwargs):return [DetectionResult([[8,8,20,20]])]
            views=[(t,np.eye(4),np.zeros((32,32,3),np.uint8),None) for t in (.8,1.,1.2)]
            with patch.dict(sys.modules,{'ultralytics':SimpleNamespace(YOLO=lambda _:Model())}), \
                 patch.object(vr,'iterate_views',return_value=views), \
                 patch.object(vr,'estimate_floor',side_effect=lambda xyz,traj:np.full(len(xyz),-1.)), \
                 patch.object(sys,'argv',['clean','--map',str(source),'--bag','unused','--config',str(path)]):
                vr.main()
            retained=vr.read_pcd(source/'global_clean.pcd')[1]
            self.assertEqual(retained['intensity'].tolist(),[9.])
            self.assertEqual((source/'global.pcd').read_bytes(),original)
            report=json.loads(next((source/'clean_diagnostics').glob('*/summary.json')).read_text())
            self.assertEqual(report['vehicle_perception'],'detect')
            self.assertEqual(report['detection_association']['accepted_components'],3)
            self.assertEqual(report['vote_rejections']['seed_voxels'],9)


if __name__=='__main__':unittest.main()
