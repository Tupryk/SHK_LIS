#!/usr/bin/python3

import os
import glob
import signal
from mesh_helper import *
from test_h5 import *

files = sorted(glob.glob('*_description/meshes/*/*.ply'))

signal.signal(signal.SIGALRM, timeout)

for file in files:
    if file[-5]=='-':
        continue
    
    print('file: ', file)

    ### load
    mesh = load_mesh(file)
    if mesh==None:
        continue

    ### repair
    try:
        trimesh.constants.tol.merge = 1e-6
        mesh.process(validate=True)
        trimesh.repair.fill_holes(mesh)
        trimesh.repair.fix_inversion(mesh, multibody=True)
    except Exception as e:
        print('  --- repair failed ---', e)
        continue
    print('  watertight:', mesh.is_watertight)
    print('  oriented:', mesh.is_winding_consistent)

    #mesh.visual.to_color()

    ## export
    filebase = os.path.splitext(file)[0]
    print('  exporting:', filebase)

    try:
        #export_mesh_h5(mesh, 'tmp/'+filebase+'.h5')
        mesh.export(filebase+'-.ply')
        os.system('meshTool ' + filebase+'-.ply -hide'
                  ' && mv z.ply ' + filebase + '-.ply' )
    except:
        print('==== ply export failed ====')
    #export_mesh(mesh, filebase+'.mesh')
    #test_read(filebase+'-.mesh')
