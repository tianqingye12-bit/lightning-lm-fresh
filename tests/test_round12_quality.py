"""Actual projection/voting/export regressions with scored synthetic model output."""
import csv
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
import clean_map_vehicles as vr
from clean_map_keyframes import run_keyframe_clean
from segmentation_quality import (ScoredInstances, ScoredInstanceCache, quality_options,
                                  instance_cache, associate_scored_instances)
from semantic_evidence import decide, PositiveBaseline
from configure_quality_round12 import configure, write_profile
from evaluate_quality import instance_counts, plane_metrics, evaluate_segmentation


def config():
    cfg = yaml.safe_load((ROOT/'config/m2dgr_clean_instances.yaml').read_text())
    cfg.update(width=32, height=32, K=[10.,0.,16.,0.,10.,16.,0.,0.,1.], distortion=[0.]*5,
               T_cam_lidar=np.eye(4).ravel().tolist(), mask_dilate_px=1, cache={'enabled':False},
               diagnostics={'max_overlays':1}, semantic_evidence={'enabled':True,'write_uncertain':True},
               segmentation_quality={'enabled':True},
               storage={'compress_keyframes':False,'write_global_clean':True,'write_removed_points':True})
    cfg['instance_filter']['min_depth_pixels'] = 1
    return cfg


def prediction(score=.9):
    masks = np.zeros((1,32,32),bool); masks[0,14:19,14:19] = True
    return ScoredInstances(masks,[2],[score])


class QualityTests(unittest.TestCase):
    def test_core_halo_and_low_confidence_never_become_background(self):
        cfg=config(); scene=np.full((32,32),2.)
        u=np.array([16,18,19,23]);v=np.full(4,16);z=np.full(4,2.);visible=np.ones(4,bool)
        hit,bg,_,score=associate_scored_instances(prediction(),scene,u,v,z,visible,cfg)
        np.testing.assert_array_equal(hit,[1,0,0,0]);np.testing.assert_array_equal(bg,[0,0,0,1])
        np.testing.assert_allclose(score,[.9,0,0,0])
        hit,bg,_,_=associate_scored_instances(prediction(.3),scene,u,v,z,visible,cfg)
        self.assertFalse(hit.any());np.testing.assert_array_equal(bg,[0,0,0,1])

    def test_overlap_max_score_not_sum_class_threshold_and_no_depth(self):
        cfg=config();p=prediction();p=ScoredInstances(np.repeat(p.masks,2,axis=0),[2,7],[.8,.9])
        cfg['segmentation_quality']['class_confidence']={7:.95}
        args=(np.full((32,32),2.),np.array([16]),np.array([16]),np.array([2.]),np.array([True]),cfg)
        hit,_,_,score=associate_scored_instances(p,*args)
        self.assertTrue(hit[0]);self.assertAlmostEqual(float(score[0]),.8,places=6)
        cfg['segmentation_quality']['class_confidence']={}
        self.assertAlmostEqual(float(associate_scored_instances(p,*args)[3][0]),.9,places=6)
        args=(np.full((32,32),np.inf),*args[1:])
        hit,bg,_,_=associate_scored_instances(p,*args)
        self.assertFalse(hit[0]);self.assertFalse(bg[0])

    def test_thin_mask_abstains_without_expanding_into_background(self):
        cfg=config();m=np.zeros((1,32,32),bool);m[0,16,14:20]=True
        p=ScoredInstances(m,[2],[.9])
        hit,bg,_,_=associate_scored_instances(p,np.full((32,32),2.),np.array([16]),np.array([16]),
                                              np.array([2.]),np.array([True]),cfg)
        self.assertFalse(hit[0]);self.assertFalse(bg[0])

    def test_minimum_views_mean_score_weighted_ratio_and_veto(self):
        cfg=config();cfg['semantic_evidence']['min_positive_baseline_m']=0
        baseline=PositiveBaseline(5,cfg)
        # low mean score; clear target; only one view; weighted-ratio failure; raw bg veto.
        visible=np.array([3,3,1,5,6]);positive=np.array([3,3,1,3,3]);background=np.array([0,0,0,2,3])
        _,remove,_,counts=decide(visible,positive,background,cfg,baseline,
                                positive_score=np.array([1.35,2.7,.9,1.8,2.7]))
        np.testing.assert_array_equal(remove,[0,1,0,0,0]);self.assertEqual(counts['blocked_confidence'],2)
        with self.assertRaises(ValueError):decide(visible,positive,background,cfg,baseline)

    def test_scored_cache_metadata_empty_corruption_and_schema_isolation(self):
        with tempfile.TemporaryDirectory() as tmp:
            cfg=config();image=np.zeros((32,32,3),np.uint8)
            cache=instance_cache(tmp,{'schema':2},cfg);key=cache.key(image)
            cache.put(key,prediction(),1);restored,n=cache.get(key,(32,32))
            self.assertEqual(n,1);np.testing.assert_array_equal(restored.classes,[2])
            np.testing.assert_allclose(restored.scores,[.9]);np.testing.assert_array_equal(restored.masks,prediction().masks)
            cfg['segmentation_quality']['enabled']=False
            self.assertNotEqual(key,instance_cache(tmp,{'schema':2},cfg).key(image))
            cache.put(key,ScoredInstances(np.empty((0,32,32),bool),[],[]),0)
            self.assertEqual(cache.get(key,(32,32))[1],0)
            np.savez(Path(tmp)/key[:2]/(key+'.npz'),shape=[1,32,32],bits=np.zeros(128,np.uint8))
            self.assertIsNone(cache.get(key,(32,32)));self.assertEqual(cache.corrupt,1)

    def test_real_inference_retains_metadata_and_rejects_mismatch(self):
        def tensor(a):return SimpleNamespace(cpu=lambda:SimpleNamespace(numpy=lambda:np.asarray(a)))
        boxes=SimpleNamespace(cls=tensor([2]),conf=tensor([.8]))
        result=SimpleNamespace(masks=SimpleNamespace(data=tensor(prediction().masks)),boxes=boxes)
        voter=vr.VehicleVoter(config(),np.empty((0,3)),lambda *a,**kw:[result],'cpu')
        pred,n=voter.infer_mask(np.zeros((32,32,3),np.uint8))
        self.assertEqual(n,1);np.testing.assert_array_equal(pred.classes,[2]);np.testing.assert_allclose(pred.scores,[.8])
        boxes.conf=tensor([])
        with self.assertRaises(ValueError):voter.infer_mask(np.zeros((32,32,3),np.uint8))

    def test_profile_preserves_calibration_storage_and_never_overwrites(self):
        cfg=config();profile=configure(cfg,'semantic')
        for k in ('K','distortion','T_cam_lidar','time_offset','storage','yolo_model','width','height'):
            self.assertEqual(cfg[k],profile[k])
        source={'fasterlio':{'filter_size_scan':.1,'enable_icp_part':False,'imu_prediction':{'max_step_s':.1}},
                'system':{'map_export':{'save_global_pcd':False}}}
        mapped=configure(source,'mapping');self.assertEqual(mapped['system'],source['system'])
        for k,v in source['fasterlio'].items():self.assertEqual(mapped['fasterlio'][k],v)
        self.assertFalse(configure(source,'mapping',diagnostics_only=True)['fasterlio']['registration_quality']['balance'])
        with tempfile.TemporaryDirectory() as tmp:
            src,dst=Path(tmp)/'base.yaml',Path(tmp)/'new.yaml';src.write_text(yaml.safe_dump(cfg))
            write_profile(src,dst,'semantic')
            with self.assertRaises(FileExistsError):write_profile(src,dst,'semantic')
        for invalid in ({'enabled':1},{'core_erode_px':-1},{'class_confidence':{2:float('nan')}},
                        {'min_instance_confidence':.1},{'unknown':True}):
            bad=config();bad['segmentation_quality'].update(invalid)
            with self.subTest(invalid=invalid),self.assertRaises(ValueError):quality_options(bad)

    def exercise_export(self, mode, score):
        with tempfile.TemporaryDirectory() as tmp:
            d=Path(tmp);frames=d/'keyframe_clouds';frames.mkdir()
            header=['VERSION .7','FIELDS x y z intensity','SIZE 4 4 4 4','TYPE F F F F',
                    'COUNT 1 1 1 1','WIDTH 5','HEIGHT 1','POINTS 5','DATA binary']
            cloud=np.array([(0,0,2,7),(.4,0,2,8),(1,0,2,9),(0,0,5,10),(float('nan'),0,2,11)],
                dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('intensity','<f4')])
            vr.write_pcd(d/'global.pcd',header,cloud);vr.write_pcd(frames/'0.pcd',header,cloud)
            originals=[p.read_bytes() for p in (d/'global.pcd',frames/'0.pcd')]
            np.savetxt(d/'keyframes.txt',[[0,1,0,0,0,0,0,0,1]])
            (frames/'index.csv').write_text('id,timestamp,file\n0,1,0.pcd\n')
            (d/'map_frames.yaml').write_text('format_version: 2\n')
            cfg=config();cfg['clean_mode']=mode;file=d/'cfg.yaml';file.write_text(yaml.safe_dump(cfg))
            pose=np.eye(4);moving=pose.copy();moving[0,3]=.06
            views=[(t,p,np.zeros((32,32,3),np.uint8),None) for t,p in zip([.8,1.,1.2],[pose,moving,moving])]
            with patch.dict(sys.modules,{'ultralytics':SimpleNamespace(YOLO=lambda _:SimpleNamespace(task='segment'))}), \
                 patch.object(vr,'iterate_views',return_value=views), \
                 patch.object(vr.VehicleVoter,'infer_mask',return_value=(prediction(score),1)), \
                 patch.object(vr,'estimate_floor',return_value=np.full(4,-1.)), \
                 patch.object(vr,'components',side_effect=AssertionError('must not grow')), \
                 patch.object(vr,'load_frame_contract',return_value=(np.eye(4),'imu_v2')):
                if mode=='keyframes':
                    run_keyframe_clean(SimpleNamespace(map=tmp,bag='unused',device='cpu',no_cache=True,rviz=False),cfg,'imu_v2')
                else:
                    with patch.object(sys,'argv',['clean','--map',tmp,'--bag','unused','--config',str(file)]):vr.main()
            run=next((d/'clean_diagnostics').iterdir());output=run if mode=='keyframes' else d
            removed=vr.read_pcd(output/'global_vehicles.pcd')[1]
            self.assertEqual(len(removed),int(score>=.55))
            if len(removed):self.assertEqual(float(removed['intensity'][0]),7)
            summary=json.loads((run/'summary.json').read_text())
            self.assertEqual(summary['evidence']['blocked_confidence'],int(score<.55))
            self.assertEqual(summary['evidence']['unobserved'],1)
            self.assertEqual([p.read_bytes() for p in (d/'global.pcd',frames/'0.pcd')],originals)
            rows=list(csv.DictReader((run/'views.csv').read_text().splitlines()))
            self.assertEqual(len(rows),3);self.assertAlmostEqual(float(rows[0]['positive_confidence_sum']),score,places=6)

    def test_map_and_keyframe_exports_keep_boundaries_and_low_confidence(self):
        for mode in ('map','keyframes'):
            for score in (.45,.9):
                with self.subTest(mode=mode,score=score):self.exercise_export(mode,score)


class EvaluationTests(unittest.TestCase):
    def test_instance_match_duplicates_wrong_class_negatives_and_empty(self):
        masks=prediction().masks
        counts,pixels=instance_counts(np.repeat(masks,3,axis=0),[2,2,7],[.9,.8,.7],masks,[2])
        self.assertEqual(counts,{2:dict(tp=1,fp=1,fn=0),7:dict(tp=0,fp=1,fn=0)})
        self.assertEqual(pixels,dict(tp=25,fp=0,fn=0))
        counts,_=instance_counts(np.empty((0,32,32),bool),[],[],masks,[2]);self.assertEqual(counts[2]['fn'],1)
        counts,_=instance_counts(masks,[2],[.9],np.empty((0,32,32),bool),[]);self.assertEqual(counts[2]['fp'],1)

    def test_fixed_reference_detects_bias_and_reports_geometry_loss(self):
        xy=np.array([(i*.2,j*.2,0.) for i in range(4) for j in range(4)])
        spec=[dict(name='wall',normal=[0,0,2],offset=0,min=[-1,-1,-1],max=[1,1,1],coverage_cell_m=.1)]
        exact=plane_metrics(xy,spec)[0];biased=plane_metrics(xy+[0,0,.12],spec)[0]
        self.assertEqual(exact['rmse_m'],0);self.assertAlmostEqual(biased['rmse_m'],.12)
        self.assertAlmostEqual(biased['thickness_p95_p05_m'],0)
        self.assertLess(plane_metrics(xy[:2],spec)[0]['occupied_plane_cells'],exact['occupied_plane_cells'])
        self.assertIsNone(plane_metrics(np.empty((0,3)),spec)[0]['rmse_m'])

    def test_manifest_evaluation_uses_fixed_predictions_without_model(self):
        with tempfile.TemporaryDirectory() as tmp:
            d=Path(tmp);cv2.imwrite(str(d/'image.png'),np.zeros((32,32,3),np.uint8));p=prediction(.45)
            np.savez(d/'truth.npz',masks=p.masks,classes=p.classes)
            np.savez(d/'pred.npz',masks=p.masks,classes=p.classes,scores=p.scores)
            (d/'samples.json').write_text(json.dumps([dict(image='image.png',truth='truth.npz',prediction='pred.npz')]))
            cfg=config();cfg['segmentation_quality']['min_instance_confidence']=.5
            (d/'cfg.yaml').write_text(yaml.safe_dump(cfg))
            args=SimpleNamespace(config=str(d/'cfg.yaml'),manifest=str(d/'samples.json'),iou=.5,device='cpu')
            report=evaluate_segmentation(args)
            self.assertEqual(report['overall']['fn'],1);self.assertEqual(report['overall']['recall'],0)
            self.assertIsNone(report['overall']['precision'])
            self.assertEqual(len(report['frames'][0]['prediction_sha256']),64)


if __name__=='__main__':unittest.main()
