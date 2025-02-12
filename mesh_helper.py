import os
import numpy as np
#import matplotlib.pyplot as plt
import trimesh
import base64
import yaml
import h5py
import matplotlib.pyplot as plt

def write_arr(X, fil, type='float32'):
    data = (type, list(X.shape), )
    fil.write(f'[ "{type}", {list(X.shape)}, "')
    fil.write(base64.b64encode(X.astype(type)).decode('utf-8'))
    fil.write('" ]')

def conv_tuple_arr(data_tuple):
    X = np.frombuffer(base64.decodebytes(bytearray(data_tuple[2].encode('utf-8'))), dtype=data_tuple[0])
    X = X.reshape(data_tuple[1])
    return X

class MeshHelper():
    def __init__(self, file):
        self.failed = False
        self.inertiaIsDiagonal = False
        self.load(file)
        

    def load(self, file):
        print('=== file: ', file)
        self.mesh = trimesh.load(file, force='mesh')
        self.filebase = os.path.splitext(file)[0]
        try:
            scene_or_mesh = trimesh.load(file, force='mesh')
        except Exception as e:
            print(e)
            print('  load failed:', file)
            return None

        if isinstance(scene_or_mesh, trimesh.Scene):
            if len(scene_or_mesh.geometry) == 0:
                self.mesh = None
                self.failed = True
            else:
                # we lose texture information here
                self.mesh = trimesh.util.concatenate(
                    tuple(trimesh.Trimesh(vertices=g.vertices, faces=g.faces)
                        for g in scene_or_mesh.geometry.values()))
        else:
            assert(isinstance(scene_or_mesh, trimesh.Trimesh))
            self.mesh = scene_or_mesh

    def autoScale(self):
        scale = 1
        #if self.mesh.scale > 10000:
        #    scale = 1e-5
        #elif self.mesh.scale > 1000:
        #    scale = 1e-4
        if self.mesh.scale > 200:
            scale = 1e-3
        elif self.mesh.scale > 20:
            scale = 1e-2
        elif self.mesh.scale > 2:
            scale = 1e-1
        #translate = -mesh.centroid
        translate = -.5*(self.mesh.bounds[0]+self.mesh.bounds[1])
        matrix = np.eye(4)
        matrix[0:3, 3] = translate
        matrix[0:3, 0:4] *= scale
        self.mesh.apply_transform(matrix)

    def transformInertia(self, verbose=False):
        print("prior COM" , self.mesh.center_mass)
        print("prior inertia\n" , self.mesh.moment_inertia)


        U, D, V = np.linalg.svd(self.mesh.moment_inertia)

        # Ensure proper rotation  
        if np.linalg.det(V) < 0:
            V[:, -1] *= -1

        matrix = np.eye(4)
        matrix[0:3, 3] = V @ (-self.mesh.center_mass)  # Move center of mass to origin
        matrix[0:3, 0:3] = V  # Rotate the mesh to align with principal axes

        self.mesh.apply_transform(matrix)

        if verbose:
            print("COM after transformation:" , self.mesh.center_mass)
            print("Inertia after transfromation:\n" , self.mesh.moment_inertia)
        self.inertiaIsDiagonal = True

        return np.linalg.inv(matrix)

    def repair(self):
        try:
            trimesh.constants.tol.merge = 1e-6
            self.mesh.process(validate=True)
            trimesh.repair.fill_holes(self.mesh)
            trimesh.repair.fix_inversion(self.mesh, multibody=True)
        except Exception as e:
            print('  --- repair failed ---', e)
            self.failed = True
            exit(0) # this might be a trimesh bug: change order within mesh.process method

        if self.mesh.visual != None:
            self.mesh.visual.uv = np.array([[0,0]])

    def export_ply(self):
        print('  exporting as ply and mesh')
        self.mesh.export(self.filebase+'-.ply')

    def export_mesh(self, include_mass_properties=True):
        with open(self.filebase+'.mesh', 'w', encoding='utf-8') as fil:
            fil.write('{\nV: ')
            write_arr(self.mesh.vertices, fil)
            assert self.mesh.faces.shape[1]==3, 'can only export triangle meshes'
            fil.write(', \nT: ')
            if(self.mesh.vertices.shape[0]<65535):
                write_arr(self.mesh.faces, fil, 'uint16')
            else:
                write_arr(self.mesh.faces, fil, 'uint32')
            if include_mass_properties:
                fil.write(f', \nmass: {self.mesh.mass}')
                # fil.write(f', \nvolume: {self.mesh.volume}')
                fil.write(f', \ncom: {self.mesh.center_mass.tolist()}')
                if self.inertiaIsDiagonal:
                    fil.write(f', \ninertia: {np.diagonal(self.mesh.moment_inertia).tolist()}')
                else:
                    fil.write(f', \ninertia: {self.mesh.moment_inertia.reshape([9]).tolist()}')
            fil.write('\n}')

    def export_scene(self, convex=False):
        with open(self.filebase+'.g', 'w', encoding='utf-8') as fil:
            if self.inertiaIsDiagonal:
                if convex:
                    fil.write(f'obj: {{ X:[0., 0., 1.], mesh_decomp: <{self.filebase}.h5>, mass: {self.mesh.center_mass.tolist()}, inertia: {np.diagonal(self.mesh.moment_inertia).tolist()} }}\n')
            else:
                fil.write(f'obj: {{ X:[0., 0., 12.], mass: {self.mesh.center_mass.tolist()}, inertia: {self.mesh.moment_inertia.reshape([9]).tolist()} }}\n')
            fil.write(f'obj_mesh (obj): {{ mesh: <{self.filebase}.h5> }}\n')
            #fil.write(f'obj_points (obj): {{ mesh_points: <{self.filebase}.h5>, color: [1 1 0], size: [2.] }}\n')
            return self.filebase+'.g'




    def createPoints(self):
        self.pts, faces = trimesh.sample.sample_surface(self.mesh, 20000)
        self.normals = self.mesh.face_normals[faces]
        #bary = trimesh.triangles.points_to_barycentric(self.mesh.triangles[faces], pts)
        #normals = trimesh.unitize((self.mesh.vertex_normals[self.mesh.faces[faces]] *
        #                          trimesh.unitize(bary).reshape((-1, 3, 1))).sum(axis=1))
    

    def createDecomposition(self):

        convex_hulls = self.mesh.convex_decomposition()
        
        self.decomp_parts = [0]
        self.decomp_vertices = np.asarray(convex_hulls[0].vertices)
        self.decomp_faces = np.asarray(convex_hulls[0].faces)
        vert_len = len(convex_hulls[0].vertices) 
        for i, part in enumerate(convex_hulls[1:]):                

            self.decomp_vertices = np.concatenate([self.decomp_vertices, part.vertices]) 
            self.decomp_faces = np.concatenate([self.decomp_faces, part.faces + vert_len])  
            self.decomp_parts.append(len(part.vertices)+self.decomp_parts[i])

            vert_len+=len(part.vertices)
        
        self.decomp_parts = np.asarray(self.decomp_parts)
        self.decomp_colors = [0,255,255]  # fixed blue for now

    def createDecomposition_lowlevel(self):
        ### create decomposition
        ret = os.system('ry-meshTool.sh ' + self.filebase+'.mesh' + ' -decomp -hide -quiet'
                        ' && mv z.arr ' + self.filebase+'.decomp' )
        if ret>0:
            print('  --- decomposition failed --- return:', ret)
            self.failed=True
            return

        ### load decomposition
        with open(self.filebase+'.decomp', 'r') as fil:
            decomp = yaml.safe_load(fil)
        self.decomp_vertices = conv_tuple_arr(decomp['V'])
        self.decomp_faces = conv_tuple_arr(decomp['T'])
        self.decomp_colors = conv_tuple_arr(decomp['C'])
        self.decomp_parts = conv_tuple_arr(decomp['cvxParts'])



    def export_h5(self, convex=False):
        with h5py.File(self.filebase+'.h5', 'w') as fil:
            fil.create_dataset('mesh/vertices', data=self.mesh.vertices, dtype='float32')
            assert self.mesh.faces.shape[1]==3, 'can only export triangle meshes'
            if(self.mesh.vertices.shape[0]<65535):
                fil.create_dataset('mesh/faces', data=self.mesh.faces, dtype='uint16')
            else:
                fil.create_dataset('mesh/faces', data=self.mesh.faces, dtype='uint32')
            
            try:
                colors = np.asarray(self.mesh.visual.vertex_colors)[:,0:3]
                fil.create_dataset('mesh/colors', data=colors, dtype='uint8')
            except:
                print("no colors for:", self.filebase)

            fil.create_dataset('points/vertices', data=self.pts, dtype='float32')
            #fil.create_dataset('points/normals', data=self.normals, dtype='float32')
            if (convex):
                fil.create_dataset('decomp/vertices', data=self.decomp_vertices, dtype='float32')
                assert self.decomp_faces.shape[0]<65535
                fil.create_dataset('decomp/faces', data=self.decomp_faces, dtype='uint16')
                fil.create_dataset('decomp/colors', data=self.decomp_colors, dtype='uint8')
                assert self.decomp_parts.shape[ 0]<65535
                fil.create_dataset('decomp/parts', data=self.decomp_parts, dtype='uint16')
            fil.create_dataset('inertia/mass', data=[self.mesh.mass], dtype='float32')
            fil.create_dataset('inertia/com', data=self.mesh.center_mass, dtype='float32')
            if self.inertiaIsDiagonal:
                fil.create_dataset('inertia/tensor', data=np.diagonal(self.mesh.moment_inertia), dtype='float32')
            else:
                fil.create_dataset('inertia/tensor', data=self.mesh.moment_inertia, dtype='float32')
        
    
def timeout(signum, frame):
    raise Exception("timeout handler")

def get_sdf(mesh, N=30):
    #scale_mesh(mesh, .1)
    # get bounds
    bounds = mesh.bounds.copy()
    size = bounds[1] - bounds[0]
    # enlarge by 10%
    bounds[0] = bounds[0] - .1 * size
    bounds[1] = bounds[1] + .1 * size
    size = bounds[1] - bounds[0]
    # decide on a voxel size
    voxelSize = 1. / (N * np.power(np.prod(size), -1. / 3))
    gridDim = (size / voxelSize).astype(int) + 2
    # change bounds[1] to be on the grid
    bounds[1] = bounds[0] + voxelSize * (gridDim - 1)
    print('  sdf voxel:', voxelSize, 'dim:', gridDim, 'mem:', np.prod(gridDim))
    # create grid
    x_range = np.linspace(bounds[0, 0], bounds[1, 0], num=gridDim[0])
    y_range = np.linspace(bounds[0, 1], bounds[1, 1], num=gridDim[1])
    z_range = np.linspace(bounds[0, 2], bounds[1, 2], num=gridDim[2])
    grid = np.stack(np.meshgrid(x_range, y_range, z_range, indexing='ij'), axis=-1)
    # get sdf
    #sdf = -mesh.nearest.signed_distance(grid).reshape(gridDim)
    sdf = np.empty(gridDim)
    for z in range(0, sdf.shape[2]):
        print('\r  slice', z, end=' ', flush=True)
        sdf[:,:,z] = -mesh.nearest.signed_distance(grid[:,:,z,:].reshape(-1, 3)).reshape(gridDim[:2])
    print('- done')
    #scale_mesh(mesh, 10.)
    #bounds *= 10.
    return [sdf, bounds]

def display_sdf(sdf):
    ax = plt.subplot()
    cax = plt.axes([0.85, 0.1, 0.075, 0.8])
    for z in range(0, sdf.shape[2]):
        print(z)
        print(sdf[:, :, z])
        im = ax.imshow(sdf[:, :, z])
        plt.colorbar(im, cax=cax)
        plt.pause(.2)


        