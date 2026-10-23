#!/usr/bin/env python3
"""Reusable, loopback-only manual sim checks. Never starts external GLIM."""
import argparse
import ctypes as C
import json
import os
from pathlib import Path
import re
import shutil
import signal
import stat
import struct
import subprocess
import sys
import time

import yaml

ROOT = Path(__file__).resolve().parents[1]


def prepare(args):
    run = args.run_dir.resolve()
    if run.exists() and any(run.iterdir()):
        raise ValueError('Use a new empty run directory; existing evidence is preserved')
    run.mkdir(parents=True, exist_ok=True)
    for folder in ('sim/build', 'deploy/build', 'deploy/config', 'ros_logs'):
        (run/folder).mkdir(parents=True, exist_ok=True)
    shutil.copy2(ROOT/'simulate/build/unitree_mujoco', run/'sim/build/unitree_mujoco')
    shutil.copy2(ROOT/'deploy/robots/g1/build/g1_ctrl', run/'deploy/build/g1_ctrl')
    os.mkfifo(run/'joystick')
    sim = yaml.safe_load((ROOT/'simulate/config.yaml').read_text())
    sim.update(robot_scene=str(ROOT/'src/assets/robots/unitree_g1/xmls/scene_g1_table_box_glim.xml'),
               joystick_device=str(run/'joystick'), interface='lo', domain_id=args.domain_id)
    sim['frame_pose_list']['aruco']['display'] = False
    sim['mid360']['scan_pattern'] = str(ROOT/'simulate/assets/livox/mid360.csv')
    (run/'sim/config.yaml').write_text(yaml.safe_dump(sim, sort_keys=False))
    deploy = yaml.safe_load((ROOT/'deploy/robots/g1/config/config.yaml').read_text())
    for value in deploy['FSM'].values():
        if 'policy_dir' in value:
            value['policy_dir'] = str(ROOT/'deploy/robots/g1'/value['policy_dir'])
    deploy['FSM']['Navigation']['endpoint'] = 'ipc://'+str(run/'navigation.sock')
    deploy['FSM']['OnlineMimic']['endpoint'] = 'ipc://'+str(run/'deploy.sock')
    (run/'deploy/config/config.yaml').write_text(yaml.safe_dump(deploy, sort_keys=False))
    commands = {
        'simulator': [str(run/'sim/build/unitree_mujoco'), '--network', 'lo', '--domain-id', str(args.domain_id)],
        'controller': [str(run/'deploy/build/g1_ctrl'), '--network', 'lo', '--environment', 'sim',
                       '--domain-id', str(args.domain_id), '--navigation-localization-source', args.localization,
                       '--sensor-model', str(ROOT/'config/g1_sensor_frames.yaml')],
        'aruco': [str(args.sensor_python.absolute()), str(ROOT/'aruco/frame_pose_publisher.py'),
                  '--environment', 'sim', '--network', 'lo', '--domain-id', str(args.domain_id), '--config', str(run/'sim/config.yaml')],
    }
    if args.localization == 'glim':
        commands['controller'] += ['--glim-config',str(args.glim_config.resolve())]
    else:
        commands['simulator'] += ['--disable-lidar']
    manifest = dict(commands=commands, root=str(ROOT), localization=args.localization,
                    glim_config=str(args.glim_config.resolve()) if args.localization == 'glim' else None,
                    commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
                    environment={'ROS_DOMAIN_ID':str(args.ros_domain_id), 'ROS_LOCALHOST_ONLY':'1',
                                 'ROS_LOG_DIR':str(run/'ros_logs')},
                    truth_uses=['evaluation topics only'] + (['robot localization via rt/sportmodestate'] if args.localization=='simulator' else []))
    (run/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(manifest,indent=2))


def joystick(run, mode):
    path=run/'joystick'
    if not stat.S_ISFIFO(path.stat().st_mode):
        raise ValueError('Expected the prepared simulator FIFO, not a device')
    events={'stand':[(2,2,32767),(2,7,-32767)],
            'velocity':[(2,5,32767),(1,0,1)], 'passive':[(2,2,32767),(1,1,1)]}
    fd=os.open(path,os.O_WRONLY|os.O_NONBLOCK)
    try:
        for kind,number,value in events[mode]:
            os.write(fd,struct.pack('IhBB',0,value,kind,number)); time.sleep(.1)
        time.sleep(.4)
        for kind,number,_ in reversed(events[mode]):
            os.write(fd,struct.pack('IhBB',0,0,kind,number)); time.sleep(.1)
    finally:
        os.close(fd)


def key(run, name):
    if name=='9':
        log=(run/'controller.log').read_text()
        states=re.findall(r'FSM: (?:Start (\w+)|Change state from \w+ to (\w+))',log)
        if not states or (states[-1][0] or states[-1][1]) != 'Velocity':
            raise ValueError('Hang toggle requires a logged active Velocity policy first')
    lines=subprocess.check_output(['wmctrl','-l'],text=True).splitlines()
    windows=[line.split()[0] for line in lines if 'MuJoCo :' in line]
    if len(windows)!=1:
        raise ValueError('Expected exactly one MuJoCo window; select the correct window manually')
    subprocess.run(['wmctrl','-ia',windows[0]],check=True)
    x=C.CDLL('libX11.so.6'); xt=C.CDLL('libXtst.so.6')
    x.XOpenDisplay.argtypes=[C.c_char_p]; x.XOpenDisplay.restype=C.c_void_p
    x.XStringToKeysym.argtypes=[C.c_char_p]; x.XStringToKeysym.restype=C.c_ulong
    x.XKeysymToKeycode.argtypes=[C.c_void_p,C.c_ulong]; x.XKeysymToKeycode.restype=C.c_uint
    x.XFlush.argtypes=[C.c_void_p]; x.XCloseDisplay.argtypes=[C.c_void_p]
    xt.XTestFakeKeyEvent.argtypes=[C.c_void_p,C.c_uint,C.c_int,C.c_ulong]
    display=x.XOpenDisplay(None)
    if not display: raise RuntimeError('Cannot open X display')
    try:
        time.sleep(.2)
        code=x.XKeysymToKeycode(display,x.XStringToKeysym(name.encode()))
        xt.XTestFakeKeyEvent(display,code,1,0); x.XFlush(display); time.sleep(.08)
        xt.XTestFakeKeyEvent(display,code,0,0); x.XFlush(display)
    finally: x.XCloseDisplay(display)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='command',required=True)
    p=sub.add_parser('prepare'); p.add_argument('run_dir',type=Path)
    p.add_argument('--localization',choices=['glim','simulator'],required=True)
    p.add_argument('--glim-config',type=Path,default=ROOT/'glim/config')
    p.add_argument('--domain-id',type=int,default=83); p.add_argument('--ros-domain-id',type=int,default=84)
    p.add_argument('--sensor-python',type=Path,default=ROOT/'.venv-sensors/bin/python')
    p=sub.add_parser('run'); p.add_argument('run_dir',type=Path); p.add_argument('component',choices=['simulator','controller','aruco'])
    p=sub.add_parser('joystick'); p.add_argument('run_dir',type=Path); p.add_argument('mode',choices=['stand','velocity','passive'])
    p=sub.add_parser('stop'); p.add_argument('run_dir',type=Path); p.add_argument('component',choices=['simulator','controller','aruco','all'],default='all',nargs='?')
    p=sub.add_parser('key'); p.add_argument('run_dir',type=Path); p.add_argument('key',choices=['9','BackSpace','space'])
    args=parser.parse_args(); args.run_dir=args.run_dir.resolve()
    if args.command=='prepare': prepare(args)
    elif args.command=='joystick': joystick(args.run_dir,args.mode)
    elif args.command=='key': key(args.run_dir,args.key)
    elif args.command=='stop':
        names=['aruco','controller','simulator'] if args.component=='all' else [args.component]
        for name in names:
            file=args.run_dir/(name+'.process.json')
            if not file.exists(): continue
            record=json.loads(file.read_text())
            proc=Path('/proc')/str(record['pid'])/'stat'
            if proc.exists() and proc.read_text().split()[21]==record['start']:
                os.kill(record['pid'],signal.SIGTERM)
    else:
        manifest=json.loads((args.run_dir/'manifest.json').read_text())
        from run_online_deploy import run_processes
        os.environ.update(manifest['environment'])
        record={'pid':os.getpid(),'start':Path('/proc/self/stat').read_text().split()[21]}
        (args.run_dir/(args.component+'.process.json')).write_text(json.dumps(record)+'\n')
        with (args.run_dir/(args.component+'.log')).open('a',buffering=1) as log:
            # run_processes owns shutdown/terminal restoration. Redirect descriptors
            # so C++ subprocess output is captured as well as Python output.
            os.dup2(log.fileno(),1); os.dup2(log.fileno(),2)
            raise SystemExit(run_processes([manifest['commands'][args.component]],manifest['root']))


if __name__=='__main__': main()
