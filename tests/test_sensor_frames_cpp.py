"""Verify the C++ consumer against Python FK and validate external config parsing."""
from pathlib import Path
import json
import subprocess
import sys
import tempfile
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from robot_frames import SensorKinematics, DEFAULT_MODEL


class CppFramesTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.exe = str(Path(cls.temp.name)/'frames')
        subprocess.run(['g++', '-std=c++17', '-O1', '-pthread',
                        '-I/usr/include/eigen3', '-I'+str(ROOT/'common'),
                        '-I'+str(ROOT/'deploy/robots/g1/include'),
                        str(ROOT/'tests/sensor_frames_probe.cpp'), '-lyaml-cpp',
                        '-o', cls.exe], check=True, capture_output=True)

    @classmethod
    def tearDownClass(cls): cls.temp.cleanup()

    def test_both_consumers_have_same_frame_convention(self):
        joints = np.random.default_rng(7).uniform(-.5,.5,(50,29))
        result = subprocess.run([self.exe, str(DEFAULT_MODEL)],
                                input='\n'.join(' '.join(map(str,q)) for q in joints),
                                text=True, capture_output=True, check=True)
        actual = np.fromstring(result.stdout,sep=' ').reshape(50,2,4,4)
        fk = SensorKinematics()
        for i,q in enumerate(joints):
            for j,sensor in enumerate(['lidar','color_optical']):
                np.testing.assert_allclose(actual[i,j], fk.root_to_sensor(q,sensor), atol=1e-12)

    def test_environment_and_localization_selection(self):
        for environment, source, valid in [('sim','glim',True), ('sim','simulator',True),
                                           ('real','glim',True), ('real','simulator',False),
                                           ('sim','auto',False), ('other','glim',False)]:
            result = subprocess.run([self.exe,'--validate',environment,source],capture_output=True)
            self.assertEqual(result.returncode == 0, valid)

    def test_glim_json_comments_and_custom_filenames(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)
            (path/'config.json').write_text('// custom names\n'+json.dumps({'global':{
                'config_sensors':'sensors_custom.json','config_ros':'ros_custom.json'}}))
            (path/'sensors_custom.json').write_text(json.dumps({'sensors':{
                'T_lidar_imu':[.006,-.012,.008,0,0,0,1]}}))
            (path/'ros_custom.json').write_text(json.dumps({'glim_ros':{
                'imu_topic':'/livox/imu','points_topic':'/livox/lidar',
                'imu_frame_id':'','acc_scale':9.81}}))
            result=subprocess.run([self.exe,'--config',tmp],capture_output=True,text=True,check=True)
            self.assertEqual(result.stdout.split()[:2], ['livox_frame','odom'])
            np.testing.assert_allclose(list(map(float,result.stdout.split()[2:])),[.006,-.012,.008])
            (path/'sensors_custom.json').write_text(json.dumps({'sensors':{'T_lidar_imu':[0]*7}}))
            result=subprocess.run([self.exe,'--config',tmp],capture_output=True,text=True)
            self.assertNotEqual(result.returncode,0)


if __name__=='__main__': unittest.main()
