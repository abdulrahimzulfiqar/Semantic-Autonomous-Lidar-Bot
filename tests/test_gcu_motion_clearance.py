"""Check that the motion experiment rejects occupied, unknown and outside paths."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
import numpy as np
from PIL import Image
import yaml

spec=importlib.util.spec_from_file_location('motion',Path(__file__).resolve().parents[1]/'scripts/check_gcu_motion.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class ClearanceTests(unittest.TestCase):
    def evaluate(self, image, start=(1.,1.5,0.)):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)
            Image.fromarray(np.flipud(image)).save(p/'map.pgm')
            (p/'map.yaml').write_text(yaml.safe_dump({'image':'map.pgm','resolution':.1,'origin':[0,0,0]}))
            return module.clearance_on_path(p/'map.yaml',start)

    def test_clear_corridor(self):
        grid=np.full((32,32),254,dtype=np.uint8);grid[[0,-1],:]=0;grid[:,[0,-1]]=0
        self.assertGreater(self.evaluate(grid),.5)

    def test_wall_and_unknown_on_segment_rejected(self):
        for value in [0,205]:
            grid=np.full((32,32),254,dtype=np.uint8);grid[:,13]=value
            self.assertEqual(self.evaluate(grid),0.)

    def test_outside_rejected(self):
        self.assertEqual(self.evaluate(np.zeros((32,32),dtype=np.uint8),(-1,1,0)),0.)


if __name__=='__main__':unittest.main()
