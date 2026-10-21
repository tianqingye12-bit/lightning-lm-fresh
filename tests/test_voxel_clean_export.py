import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from voxel_pcd_stream import VoxelPcdStream
from clean_map_vehicles import read_pcd,write_pcd
from compact_clean_map import compact
from audit_clean_map import audit,vote_diagnostics

HEADER=['VERSION .7','FIELDS x y z intensity time','SIZE 4 4 4 4 8','TYPE F F F F F',
        'COUNT 1 1 1 1 1','WIDTH 4','HEIGHT 1','POINTS 4','DATA binary']
DTYPE=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('intensity','<f4'),('time','<f8')])


class ExportTests(unittest.TestCase):
    def test_cross_frame_dedup_preserves_observed_coordinates_and_every_field(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'clean.pcd'
            writer=VoxelPcdStream(path,HEADER,.1)
            first=np.array([(.01,.01,2,7,1),(-.01,0,2,8,2)],dtype=DTYPE)
            second=np.array([(.02,.01,2,9,3),(.11,0,2,10,4),(np.nan,0,2,11,5)],dtype=DTYPE)
            writer.append(first);writer.append(second);writer.finish();writer.close()
            result=read_pcd(path)[1]
            self.assertEqual(len(result),3)
            np.testing.assert_array_equal(result,np.concatenate([first[1:],first[:1],second[1:2]]))
            self.assertEqual(writer.input_count,4)
            self.assertFalse(any(Path(tmp).glob('.voxel_export_*')))

    def test_existing_result_compaction_is_not_semantic_removal(self):
        with tempfile.TemporaryDirectory() as tmp:
            source=Path(tmp)/'source';source.mkdir()
            cloud=np.array([(.01,0,2,7,1),(.02,0,2,8,2),(.11,0,2,9,3)],dtype=DTYPE)
            write_pcd(source/'global_clean.pcd',HEADER,cloud)
            (source/'summary.json').write_text(json.dumps(dict(removed_points=0)))
            original=(source/'global_clean.pcd').read_bytes()
            output=compact(source,Path(tmp)/'small',.1)
            self.assertEqual(len(read_pcd(output/'global_clean.pcd')[1]),2)
            report=json.loads((output/'summary.json').read_text())
            self.assertEqual(report['removed_points'],0)
            self.assertFalse(report['semantic_processing_rerun'])
            self.assertEqual((source/'global_clean.pcd').read_bytes(),original)
            self.assertFalse((output/'keyframe_clouds').exists())
            self.assertFalse((output/'INCOMPLETE').exists())
            self.assertGreater(audit(output)['total_bytes'],0)
            with self.assertRaises(FileExistsError):compact(source,output,.1)

    def test_vote_audit_distinguishes_insufficient_views_background_and_floor(self):
        cfg=dict(min_views=3,instance_filter={})
        result=vote_diagnostics(np.array([1,3,6,4]),np.array([1,1,3,4]),np.array([0,0,3,0]),
            cfg,np.array([False,False,False,True]),np.array([False,False,False,True]))
        self.assertEqual(result,dict(positive_points=4,blocked_min_views=1,
            blocked_positive_views=2,blocked_background=1,blocked_floor=1))

    def test_resolution_and_dtype_are_validated(self):
        with tempfile.TemporaryDirectory() as tmp:
            for value in (0,float('nan'),1):
                with self.assertRaises(ValueError):VoxelPcdStream(Path(tmp)/'a.pcd',HEADER,value)
            writer=VoxelPcdStream(Path(tmp)/'a.pcd',HEADER,.1)
            try:
                writer.append(np.empty(0,DTYPE))
                with self.assertRaises(ValueError):writer.append(np.empty(0,[('x','f8')]))
            finally:writer.close()


if __name__=='__main__':unittest.main()
