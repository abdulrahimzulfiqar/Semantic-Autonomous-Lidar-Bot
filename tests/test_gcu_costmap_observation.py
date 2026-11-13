import sys
from pathlib import Path
from types import SimpleNamespace
import unittest
import math
import numpy as np
import io,json,tarfile
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from gcu_costmap_observation import inspect,lattice_phase
from osmag_constraint_mask import ConstraintError


class NativeObservationTest(unittest.TestCase):
    def test_rolling_grid_requires_same_lattice_not_only_same_resolution(self):
        r=float(np.float32(.05864395131342248));origin=(-29.351297632367952,-20.14419727616062)
        self.assertTrue(lattice_phase(r,(origin[0]+100*r,origin[1]+90*r),r,origin)['aligned'])
        shifted=(origin[0]+100.5*r,origin[1]+90.5*r)
        self.assertFalse(lattice_phase(r,shifted,r,origin)['aligned'])
    def test_same_frame_native_mask_uses_wire_resolution_at_cell_edges(self):
        archive=Path(__file__).resolve().parents[1]/'simulation/gcu_ground_floor/phase6_diagnostics/06d_multi_map_preflight_failure/attempt02.tar.gz'
        prefix='results/multi_02/'
        with tarfile.open(archive) as evidence:
            report=json.load(evidence.extractfile(prefix+'report.json'))
            for index,item in enumerate(report['native_grid_failures'],1):
                raw=np.load(io.BytesIO(evidence.extractfile(prefix+f'native_grid_failure_{index}.npz').read()))
                snap=SimpleNamespace(mask=raw['mask'],resolution=report['snapshot']['resolution'],origin=report['snapshot']['origin'])
                result=inspect(snap,raw['native'],item['resolution'],item['origin'],(0.,0.,0.),[report['contract']['start']])
                self.assertGreater(result['required_cells'],1000)
                self.assertEqual(result['missing_cells'],0)
                self.assertEqual(result['mask_resolution'],float(np.float32(snap.resolution)))

    def test_real_doorway_grids_match_native_latest_tf_not_grid_stamp(self):
        archive=Path(__file__).resolve().parents[1]/'simulation/gcu_ground_floor/phase6_diagnostics/06d_costmap_timing/doorway04.tar.gz'
        prefix='results/doorway_04/'
        with tarfile.open(archive) as evidence:
            report=json.load(evidence.extractfile(prefix+'report.json'))
            self.assertEqual(len(report['native_grid_failures']),8)
            for index,item in enumerate(report['native_grid_failures'],1):
                raw=np.load(io.BytesIO(evidence.extractfile(prefix+f'native_grid_failure_{index}.npz').read()))
                snap=SimpleNamespace(mask=raw['mask'],resolution=report['snapshot']['resolution'],origin=report['snapshot']['origin'])
                args=(snap,raw['native'],item['resolution'],item['origin'])
                with self.assertRaisesRegex(ConstraintError,'native_mask_cells_missing'):
                    inspect(*args,item['map_from_grid_at_stamp'],[report['goal'][:2]])
                result=inspect(*args,item['map_from_grid_latest']['transform'],[report['goal'][:2]])
                self.assertGreater(result['required_cells'],2000)
                self.assertEqual(result['missing_cells'],0)

    def test_rotated_local_uses_inverse_transform_for_probes(self):
        mask=np.zeros((8,8),dtype=np.int8);mask[:,0]=100
        snap=SimpleNamespace(mask=mask,resolution=1.,origin=(0.,0.))
        # Rotate a 4x4 local grid by 90 degrees into x=0..4, y=0..4.
        local=np.zeros((4,4),dtype=np.int16);local[-1,:]=100
        report=inspect(snap,local,1.,(0.,0.),(4.,0.,math.pi/2),[(2.5,1.5)])
        self.assertEqual(report['required_cells'],4)
        local[-1,1]=0
        with self.assertRaisesRegex(ConstraintError,'native_mask_cells_missing'):inspect(snap,local,1.,(0.,0.),(4.,0.,math.pi/2),[(2.5,1.5)])

    def test_float32_resolution_is_accepted_but_arbitrary_rescaling_is_not(self):
        r=.05864395131342248;snap=SimpleNamespace(mask=np.zeros((4,4)),resolution=r,origin=(0.,0.))
        inspect(snap,np.zeros((4,4)),float(np.float32(r)),(0.,0.),(0.,0.,0.),[(r,r)],full=True)
        with self.assertRaisesRegex(ConstraintError,'resolution'):inspect(snap,np.zeros((4,4)),r+1e-6,(0.,0.),(0.,0.,0.),[(r,r)])

    def test_changed_global_origin_and_unknown_probe_are_rejected(self):
        snap=SimpleNamespace(mask=np.zeros((4,4)),resolution=1.,origin=(0.,0.))
        with self.assertRaisesRegex(ConstraintError,'geometry'):inspect(snap,np.zeros((4,4)),1.,(.1,0.),(0.,0.,0.),[(1.,1.)],full=True)
        with self.assertRaisesRegex(ConstraintError,'probe_blocked'):inspect(snap,np.full((4,4),-1),1.,(0.,0.),(0.,0.,0.),[(1.,1.)])
