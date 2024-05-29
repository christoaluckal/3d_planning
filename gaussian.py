import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal
import open3d as o3d

def landscape(x,y,mu,sig):
    xy = np.column_stack([x.flat, y.flat])

    mu = np.array([mu,mu])

    sigma = np.array([sig, sig])
    covariance = np.diag(sigma**2)

    z = multivariate_normal.pdf(xy, mean=mu, cov=covariance)

    # Reshape back to a (30, 30) grid.
    z = z.reshape(x.shape)

    return z

def createLandscape(x_lim,y_lim,gaussians,count=100,landscape_count=1,z_scale=1,inversion_p=0.5):

    x_o = np.linspace(-x_lim,x_lim,count)
    y_o = np.linspace(-y_lim,y_lim,count)


    landscapes = []
    for _ in range(landscape_count):
        x,y = np.meshgrid(x_o,y_o)
        gauss_idx = np.random.randint(0,len(gaussians))
        gauss_cx = gaussians[gauss_idx][0]
        gauss_cy = gaussians[gauss_idx][1]

        if np.random.uniform() < inversion_p:
            invert = True
        else:
            invert = False

        x -= gauss_cx
        y -= gauss_cy

        gauss_mu = gaussians[gauss_idx][2]
        gauss_sig = gaussians[gauss_idx][3]

        print(f"X:{gauss_cx},Y:{gauss_cy},mu:{gauss_mu},sig:{gauss_sig},I:{invert}")

        l = landscape(x,y,gauss_mu,gauss_sig)*z_scale*(-1 if invert else 1)

        

        x += gauss_cx
        y += gauss_cy

        x = np.clip(x,-x_lim,x_lim)
        y = np.clip(y,-y_lim,x_lim)



        

        landscapes.append(l)

    landscapes = np.array(landscapes)

    return landscapes, x, y

xl = 50
yl = 50
landscapes = 25

gaussian_list = [[np.random.uniform(-xl,xl),np.random.uniform(-yl,yl),1,np.random.uniform(2,5)] for _ in range(landscapes)]

ls,x,y = createLandscape(xl,yl,gaussians=gaussian_list,z_scale=1000,landscape_count=landscapes)

zs = np.zeros(ls[0].shape)

for i in ls:
    zs = np.add(zs,i)
    
# fig = plt.figure()
# ax = fig.add_subplot(projection='3d')
# ax.set_aspect('equal')
# ax.scatter(x,y,zs)
# plt.show()

points = np.column_stack([x.flat, y.flat, zs.flat])
geometry = o3d.geometry.PointCloud()
geometry.points = o3d.utility.Vector3dVector(points)


o3d.visualization.draw_geometries([geometry])

if input("Save?") == 'y':
    geometry.estimate_normals()

    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            geometry, depth=9)
    o3d.io.write_triangle_mesh("landscape.ply", mesh)

# o3d.visualization.draw_geometries([geometry])

# o3d.io.write_point_cloud("landscape.ply", geometry)