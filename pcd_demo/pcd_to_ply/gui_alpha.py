from functools import partial
from tkinter import *
import open3d as o3d

pcd = o3d.io.read_point_cloud(f'/home/florianpix/git_repos/iwu_thermography/py/data/bunny/down_sampled_voxel_size_0.001/bunny_down_sampled_voxel_size_0.001.ply')

vis = o3d.visualization.Visualizer()
vis.create_window()
renderer = vis.get_render_option()
renderer.mesh_show_back_face = True


def show_alpha(save=False):
    alpha = alpha_var.get() / 1000
    selection = "alpha = " + str(alpha)
    label.config(text=selection)
    vis.clear_geometries()
    alpha_shape = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
    alpha_shape.compute_vertex_normals()
    vis.add_geometry(alpha_shape)
    if save:
        o3d.io.write_triangle_mesh(f'data/alpha_{alpha}.stl', alpha_shape)


tk = Tk()
alpha_var = IntVar()
w = Scale(tk, from_=1, to=100, variable=alpha_var, length=720, orient=HORIZONTAL)
w.pack(anchor=CENTER)
Button(tk, text='Create alpha shape', command=show_alpha).pack()
Button(tk, text='Save alpha shape', command=partial(show_alpha, True)).pack()
label = Label(tk)
label.pack()

while True:
    tk.update_idletasks()
    tk.update()
    vis.poll_events()
    vis.update_renderer()
