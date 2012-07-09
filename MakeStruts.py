# vim:ts=4:et
#
#  Copyright (C) 2012 Bill Currie <bill@taniwha.org>
#  Date: 2012/2/20
#
# ##### BEGIN GPL LICENSE BLOCK #####
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software Foundation,
#  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
# ##### END GPL LICENSE BLOCK #####

# <pep8 compliant>

bl_info = {
    "name": "Strut Generator",
    "author": "Bill Currie",
    "blender": (2, 6, 3),
    "api": 35622,
    "location": "View3D > Add > Mesh > Struts",
    "description": "Add struts meshes based on selected truss meshes",
    "warning": "can get very high-poly",
    "wiki_url": "",
    "tracker_url": "",
    "category": "Add Mesh"}

import bpy
import bmesh
from bpy.props import FloatProperty,IntProperty,BoolProperty
from mathutils import Vector,Matrix,Quaternion
from math import pi, cos, sin

cossin = []

def build_cossin(n):
    global cossin
    cossin = []
    for i in range(n):
        a = 2 * pi * i / n
        cossin.append((cos(a), sin(a)))

def select_up(axis):
    if (abs(axis[0] / axis.length) < 1e-5
        and abs(axis[1] / axis.length) < 1e-5):
        up = Vector((-1, 0, 0))
    else:
        up = Vector((0, 0, 1))
    return up

def make_strut(v1, v2, id, od, n, solid, loops):
    v1 = Vector(v1)
    v2 = Vector(v2)
    axis = v2 - v1
    pos = [(0, od / 2)]
    if loops:
        pos += [((od - id) / 2, od / 2),
                (axis.length - (od - id) / 2, od / 2)]
    pos += [(axis.length, od / 2)]
    if solid:
        pos += [(axis.length, id / 2)]
        if loops:
            pos += [(axis.length - (od - id) / 2, id / 2),
                    ((od - id) / 2, id / 2)]
        pos += [(0, id / 2)]
    vps = len(pos)
    fps = vps
    if not solid:
        fps -= 1
    fw = axis.copy()
    fw.normalize()
    up = select_up(axis)
    lf = up.cross(fw)
    lf.normalize()
    up = fw.cross(lf)
    mat = Matrix((fw, lf, up))
    mat.transpose()
    verts = [None] * n * vps
    faces = [None] * n * fps
    for i in range(n):
        base = (i - 1) * vps
        x = cossin[i][0]
        y = cossin[i][1]
        for j in range(vps):
            p = Vector((pos[j][0], pos[j][1] * x, pos[j][1] * y))
            p = mat * p
            verts[i * vps + j] = p + v1
        if i:
            for j in range(fps):
                f = (i - 1) * fps + j
                faces[f] = [base + j, base + vps + j,
                            base + vps + (j + 1) % vps, base + (j + 1) % vps]
    base = len(verts) - vps
    i = n
    for j in range(fps):
        f = (i - 1) * fps + j
        faces[f] = [base + j, j, (j + 1) % vps, base + (j + 1) % vps]
    #print(verts,faces)
    return verts, faces


def make_debug_plane(mesh, edge_num, edge, od):
    v = [mesh.verts[edge.verts[0].index].co,
         mesh.verts[edge.verts[1].index].co,
         None, None]
    v[2] = v[1] + edge.z * 0.1
    v[3] = v[0] + edge.z * 0.1
    f = [edge_num * 4 + 0, edge_num * 4 + 1,
         edge_num * 4 + 2, edge_num * 4 + 3]
    return v, f
class SVert:
    def __init__(self, bmvert, bmedge):
        self.index = bmvert.index
        edges = bmvert.link_edges[:]
        edges.remove(bmedge)
        self.edges = tuple(map(lambda e: e.index, edges))

class SEdge:
    def __init__(self, bmesh, bmedge):
        self.index = bmedge.index
        self.verts = (SVert (bmedge.verts[0], bmedge),
                      SVert (bmedge.verts[1], bmedge))
        self.y = (bmesh.verts[self.verts[0].index].co
                  - bmesh.verts[self.verts[1].index].co)
        self.y.normalize()
        self.x = self.z = None

def set_edge_frame(edge, up):
    edge.x = edge.y.cross(up)
    edge.x.normalize()
    edge.z = edge.x.cross(edge.y)

def calc_edge_frame(edge, base_edge):
    baxis = base_edge.y
    if (edge.verts[0].index == base_edge.verts[0].index
        or edge.verts[1].index == base_edge.verts[1].index):
        axis = -edge.y
    elif (edge.verts[0].index == base_edge.verts[1].index
          or edge.verts[1].index == base_edge.verts[0].index):
        axis = edge.y
    else:
        raise ValueError("edges not connected")
    if baxis.dot(axis) in (-1, 1):
        # aligned axis have their up/z aligned
        up = base_edge.z
    else:
        # Get the unit vector dividing the angle (theta) between baxis and axis
        # in two equal parts
        h = (baxis + axis)
        h.normalize()
        # (cos(theta/2), sin(theta/2) * n) where n is the unit vector of the
        # axis rotating baxis onto axis
        q = Quaternion([baxis.dot (h)] + list(baxis.cross(h)))
        # rotate the base edge's up around the rotation axis (blender
        # quaternion shortcut:)
        up = q * base_edge.z
    set_edge_frame(edge, up)

def make_manifold_struts(truss_obj, id, od, segments):
    bpy.context.scene.objects.active = truss_obj
    bpy.ops.object.editmode_toggle()
    truss_mesh = bmesh.from_edit_mesh(truss_obj.data).copy()
    bpy.ops.object.editmode_toggle()
    edges = [None] * len(truss_mesh.edges)
    for i,e in enumerate(truss_mesh.edges):
        edges[i] = SEdge(truss_mesh, e)
    edge_set = set(edges)
    while edge_set:
        edge_queue=[edge_set.pop()]
        set_edge_frame(edge_queue[0], select_up(edge_queue[0].y))
        while edge_queue:
            current_edge = edge_queue.pop()
            for i in (0, 1):
                for e in current_edge.verts[i].edges:
                    edge = edges[e]
                    if edge.x != None:  #edge already processed
                        continue
                    edge_set.remove(edge)
                    edge_queue.append(edge)
                    calc_edge_frame(edge, current_edge)
    verts = []
    faces = []
    for e, edge in enumerate (edges):
        v, f = make_debug_plane(truss_mesh, e, edge, od)
        verts += v
        faces.append(f)
    return verts, faces

def make_simple_struts(truss_mesh, id, od, segments, solid, loops):
    vps = 2
    if solid:
        vps *= 2
    if loops:
        vps *= 2
    fps = vps
    if not solid:
        fps -= 1

    verts = [None] * len(truss_mesh.edges) * segments * vps
    faces = [None] * len(truss_mesh.edges) * segments * fps
    vbase = 0
    fbase = 0
    for e in truss_mesh.edges:
        v1 = truss_mesh.vertices[e.vertices[0]]
        v2 = truss_mesh.vertices[e.vertices[1]]
        v, f = make_strut(v1.co, v2.co, id, od, segments, solid, loops)
        for fv in f:
            for i in range(len(fv)):
                fv[i] += vbase
        for i in range(len(v)):
            verts[vbase + i] = v[i]
        for i in range(len(f)):
            faces[fbase + i] = f[i]
        #if not base % 12800:
        #    print (base * 100 / len(verts))
        vbase += vps * segments
        fbase += fps * segments
    #print(verts,faces)
    return verts, faces

def create_struts(self, context, id, od, segments, solid, loops, manifold):
    build_cossin(segments)

    bpy.context.user_preferences.edit.use_global_undo = False
    for truss_obj in bpy.context.scene.objects:
        if not truss_obj.select:
            continue
        truss_obj.select = False
        truss_mesh = truss_obj.to_mesh(context.scene, True, 'PREVIEW')
        if not truss_mesh.edges:
            continue
        if manifold:
            verts, faces = make_manifold_struts(truss_obj, id, od, segments)
        else:
            verts, faces = make_simple_struts(truss_mesh, id, od, segments,
                                              solid, loops)
        mesh = bpy.data.meshes.new("Struts")
        mesh.from_pydata(verts, [], faces)
        obj = bpy.data.objects.new("Struts", mesh)
        bpy.context.scene.objects.link(obj)
        obj.select = True
        obj.location = truss_obj.location
        bpy.context.scene.objects.active = obj
        mesh.update()
    bpy.context.user_preferences.edit.use_global_undo = True
    return {'FINISHED'}

class Struts(bpy.types.Operator):
    """Add one or more struts meshes based on selected truss meshes"""
    bl_idname = "mesh.generate_struts"
    bl_label = "Struts"
    bl_description = """Add one or more struts meshes based on selected truss meshes"""
    bl_options = {'REGISTER', 'UNDO'}

    id = FloatProperty(name = "Inside Diameter",
                       description = "diameter of inner surface",
                       min = 0.0,
                       soft_min = 0.0,
                       max = 100,
                       soft_max = 100,
                       default = 0.04)
    od = FloatProperty(name = "Outside Diameter",
                       description = "diameter of outer surface",
                       min = 0.001,
                       soft_min = 0.001,
                       max = 100,
                       soft_max = 100,
                       default = 0.05)
    manifold = BoolProperty(name="Manifold",
                         description="Connect struts to form a single solid.",
                         default=False)
    solid = BoolProperty(name="Solid",
                         description="Create inner surface.",
                         default=False)
    loops = BoolProperty(name="Loops",
                         description="Create sub-surf friendly loops.",
                         default=False)
    segments = IntProperty(name="Segments",
                           description="Number of segments around strut",
                           min=3, soft_min=3,
                           max=64, soft_max=64,
                           default=12)

    def execute(self, context):
        keywords = self.as_keywords ()
        return create_struts(self, context, **keywords)

def menu_func(self, context):
    self.layout.operator(Struts.bl_idname, text = "Struts", icon='PLUGIN')

def register():
    bpy.utils.register_module(__name__)
    bpy.types.INFO_MT_mesh_add.append(menu_func)

def unregister():
    bpy.utils.unregister_module(__name__)

if __name__ == "__main__":
    register()
