import json
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]


class Mid360ModelTest(unittest.TestCase):
    def test_external_extrinsic_controls_sim_sensor_origin_and_axes(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)
            (path/'config.json').write_text(json.dumps({'global':{'config_ros':'ros.json','config_sensors':'sensors.json'}}))
            (path/'ros.json').write_text(json.dumps({'glim_ros':{'imu_topic':'/livox/imu','points_topic':'/livox/lidar','acc_scale':9.81}}))
            # Include rotation as well as translation; tests more than a fixed offset.
            (path/'sensors.json').write_text(json.dumps({'sensors':{'T_lidar_imu':[.006,-.012,.008,.1,0,0,.99498743710662]}}))
            model=path/'model.xml'
            model.write_text('''<mujoco><option gravity="0 0 -9.81"/>
            <worldbody><body><geom size=".01"/>
            <site name="livox_frame" quat="0 0 1 0"/>
            <site name="livox_imu"/></body></worldbody>
            <sensor><accelerometer site="livox_imu"/></sensor></mujoco>''')
            exe=path/'probe'
            subprocess.run(['g++','-std=c++17','-O1','-I'+str(ROOT/'common'),
                            '-I'+str(ROOT/'simulate/src'),'-I'+str(ROOT/'simulate/mujoco/include'),
                            str(ROOT/'tests/mid360_model_probe.cpp'),
                            '-L'+str(ROOT/'simulate/mujoco/lib'),
                            '-Wl,-rpath,'+str(ROOT/'simulate/mujoco/lib'),
                            '-lmujoco','-lyaml-cpp','-lboost_program_options','-o',str(exe)],
                           check=True,capture_output=True)
            result=subprocess.run([str(exe),str(model),tmp],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)


if __name__=='__main__': unittest.main()
