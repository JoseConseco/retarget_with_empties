# ***** BEGIN GPL LICENSE BLOCK *****
#
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.    See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software Foundation,
# Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
# ***** END GPL LICENCE BLOCK *****
import bpy

from difflib import SequenceMatcher
from math import sqrt
from copy import copy
from mathutils import Vector
import json


bl_info = {
    "name": "Retarget using Empties",
    "author": "Jose, Maciej",
    "version": (1, 1),
    "blender": (3, 6, 0),
    "location": "Tools side panel in 3D View",
    "description": "Retarget armeture using Empties",
    "warning": "",
    "wiki_url": "",
    "category": "Animation",
    }

def is_root(bone, dimensions):
    ''' is root when not directly connected bone.head_local != bone.parent.tail_local '''
    if not bone.parent:
        return True
    #tail is the sharp part
    if (bone.head_local-bone.parent.tail_local).length > 0.001*dimensions:  # has parent, but not 'realy' connected
        return True
    return False

def find_real_root_bone(armature):
    src_chain_root_bones = [bone for bone in armature.data.bones if not bone.parent]
    return src_chain_root_bones[0] if src_chain_root_bones else None


def bone_center(bone):
    return (bone.head_local + bone.tail_local)/2

def bone_dir(bone):
    return (bone.tail_local - bone.head_local)/2


def strings_similarity(a, b):
    return SequenceMatcher(None, a, b).ratio()

def get_left_right_center_child(bone, scalex):
    center_to_bone = {child: bone_center(child) for child in bone.children}
    sorted_bones = {cbone: loc.x for cbone, loc in sorted(center_to_bone.items(), key = lambda item: item[1].x)}
    right, center, left = sorted_bones.keys()
    right_co, center_co, left_co = sorted_bones.values()
    if right_co < -0.1*scalex and abs(center_co) < 0.01*scalex and left_co > 0.1*scalex:  # ? make relative to arma size
        if len(right.children_recursive) > 1 and len(left.children_recursive) > 1:  # legs or arms - at least 3 bones shoudl have
            return left, center, right
    return None, None, None


def get_longest_chain(bone): #! will return eg. hand with longest finger
    ''' return longest child chain bones, and depth '''
    max_sub_depth = 0 # depth of max chain len for children
    max_child_chain = []
    for child in bone.children:
        child_chain_bones, child_depth = get_longest_chain(child)
        if child_depth > max_sub_depth: #? what if multiple child has same sub_chain len?
            max_sub_depth = child_depth
            max_child_chain = child_chain_bones
    return [bone]+max_child_chain, 1 + max_sub_depth


def sort_fingers(hand_bone, forward_vec):
    avg_finger_pos = []
    fingers = []
    forward = forward_vec.normalized()
    for finger_child in hand_bone.children:
        finger_bones_li, child_length = get_longest_chain(finger_child)
        avg_finger_pos.append(bone_center(finger_bones_li[0]).dot(forward))  # bigger the value the more forwared the finger is
        fingers.append(finger_bones_li)
    sorted_fingers = [finger for _, finger in sorted(zip(avg_finger_pos, fingers), reverse= True)]
    return sorted_fingers


def detect_structure(armature):
    arma_structure = {'Spine': [], 'R_Arm': [], 'L_Arm': [], 'L_Leg': [], 'R_Leg': [], 'Neck': [], 'Head': []}
    root_bone = find_real_root_bone(armature)
    if not root_bone:
        return arma_structure
    arma_size = copy(armature.dimensions.x)
    triple_child_cnt = 0  # how many times we have bone with 3child (must by legs, then arms)

    def scan_child_rec(bone, chain_name): # current_structure_name - 'Head', 'Leg', 'Neck', etc
        nonlocal triple_child_cnt, arma_size #!
        arma_structure[chain_name].append(bone) #add last bone to chain
        if not bone.children:
            return
        if len(bone.children) == 3: # leg split, or arm split or ...?
            left, center, right = get_left_right_center_child(bone, arma_size)
            if center:  # not none
                if triple_child_cnt == 0:  # must be legs
                    # scan_child_rec(right, 'R_Leg')
                    R_Leg_chain_bones, child_depth = get_longest_chain(right)
                    arma_structure['R_Leg'] = R_Leg_chain_bones

                    # scan_child_rec(left, 'L_Leg')
                    L_Leg_chain_bones, child_depth = get_longest_chain(left)
                    arma_structure['L_Leg'] = L_Leg_chain_bones

                    triple_child_cnt += 1
                    scan_child_rec(center, 'Spine')
                elif triple_child_cnt == 1:  # must be arms plus neck
                    up_vec = bone_center(arma_structure['Spine'][-1]) - bone_center(arma_structure['Spine'][0])
                    up_vec = up_vec.normalized()
                    right_vec = Vector((-1, 0, 0))
                    forward_vec = up_vec.cross(right_vec).normalized()

                    # scan_child_rec(right, 'R_Arm')
                    R_arm_chain_bones, child_depth = get_longest_chain(right)  # with one finger..
                    for arm_b in R_arm_chain_bones:
                        if len(arm_b.children) != 5:
                            arma_structure['R_Arm'].append(arm_b)
                        else:  # we got hand bone. Add fingers
                            arma_structure['R_Arm'].append(arm_b)
                            sorted_fingers = sort_fingers(arm_b, forward_vec)
                            for i, finger in enumerate(sorted_fingers):  # finger has 3 bones
                                arma_structure['R_Finger'+str(i+1)] = finger
                            break

                    # scan_child_rec(left, 'L_Arm')
                    L_arm_chain_bones, child_depth = get_longest_chain(left) #with one finger..
                    for arm_b in L_arm_chain_bones:
                        if len(arm_b.children) != 5:
                            arma_structure['L_Arm'].append(arm_b)
                        else:   # we got hand bone. Add fingers
                            arma_structure['L_Arm'].append(arm_b)
                            sorted_fingers = sort_fingers(arm_b, forward_vec)
                            for i, finger in enumerate(sorted_fingers): #finger has 3 bones
                                arma_structure['L_Finger'+str(i+1)] = finger
                            break

                    triple_child_cnt += 1
                    scan_child_rec(center, 'Neck')
            else: #triple split does not look like legs, or arms. What now?
                pass

        else: #Todo: finish filling the chain
            if chain_name in ['Spine', 'Neck']:  # for them get child bones laying on center
                for child in bone.children:
                    if abs(bone_center(child).x) < 0.01*arma_size:  # go only along center -assuming it is spine, or neck
                        if 'head' in child.name.lower():
                            arma_structure['Head'] = [child]
                            return  # finish scan on head
                        scan_child_rec(child, chain_name)  # go up spine

    scan_child_rec(root_bone, 'Spine')
    return arma_structure


class RET_OT_BuildBonesHierarchy(bpy.types.Operator):
    bl_idname = "object.build_bones_hierarchy"
    bl_label = "Build Bones Hierarchy"
    bl_description = "Build Bones Hierarchy"
    bl_options = {"REGISTER","UNDO"}

    def execute(self, context):
        ret_props = context.scene.retarget_settings
        source_arma = bpy.data.objects[ret_props.src_armature]
        target_arma = bpy.data.objects[ret_props.target_armature]
        src_chain_root_bones = detect_structure(source_arma)  # will containt chain_id: bone.name
        target_chain_root_bones = detect_structure(target_arma)

        #copy dict to CollectionProperty - ArmaHierarchyStructures
        ret_props.arma_hierarchy.clear()
        for chain_key in src_chain_root_bones.keys():
            current_hierarchy = ret_props.arma_hierarchy.add()
            current_hierarchy.name = chain_key
            src_bone_chain = src_chain_root_bones.get(chain_key)
            target_bone_chain = target_chain_root_bones.get(chain_key)
            if src_bone_chain:
                for bone in src_bone_chain:
                    new_bone = current_hierarchy.src_bones.add()
                    new_bone.name = bone.name
            if target_bone_chain:
                for bone in target_bone_chain:
                    new_bone = current_hierarchy.target_bones.add()
                    new_bone.name = bone.name

        return {"FINISHED"}


class RET_OT_CleanConstraintsHierarchy(bpy.types.Operator):
    bl_idname = "object.clean_constraints"
    bl_label = "Clean Constraints"
    bl_description = "Clean Constraints"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return context.active_object and context.active_object.type == 'ARMATURE'

    def execute(self, context):
        # get all copy raotation locatoin constraints that target empties and remove them
        for p_bone in context.active_object.pose.bones:
            for constr in reversed(p_bone.constraints):
                if constr.type in ['COPY_LOCATION', 'COPY_ROTATION']:
                    if constr.target and constr.target.type == 'EMPTY':
                        p_bone.constraints.remove(constr)

        self.report({'INFO'}, 'Cleanup sucesfull')

        return {"FINISHED"}


class RET_OT_RetargetByEmpties(bpy.types.Operator):
    bl_idname = "object.retarget_using_empties"
    bl_label = "Retarget using empties"
    bl_description = "Add copy rotation (and location) constraints to target rig, so that they will follow empties from source rig"
    bl_options = {"REGISTER","UNDO"}

    def setup_constraints(self, context, src_bone, target_bone, copy_loc, copy_rot):
        target_empty_name = src_bone.name+'T'  # empty name is from src armature
        if target_empty_name not in context.scene.objects.keys():
            self.report({'INFO'}, f'Target rig cant find bone {target_bone.name}')
            return
        else:
            target_empty = bpy.data.objects[target_empty_name]

        # add new constraiints - skip if already exists
        if copy_rot:
            # check if coy rot constraint already exists that points to target_empty, if not   create new
            for constr in target_bone.constraints:
                if constr.type == 'COPY_ROTATION' and constr.target == target_empty:
                    return
            constr = target_bone.constraints.new('COPY_ROTATION')
            constr.name = 'RetargetRot'

            constr.target = target_empty
        if copy_loc:
            for constr in target_bone.constraints:
                if constr.type == 'COPY_LOCATION' and constr.target == target_empty:
                    return
            constr = target_bone.constraints.new('COPY_LOCATION')
            constr.name = 'RetargetLoc'

            constr.target = target_empty

    def execute(self, context):
        ret_props = context.scene.retarget_settings
        source_arma = bpy.data.objects[ret_props.src_armature]
        target_arma = bpy.data.objects[ret_props.target_armature]

        if 'BoneFollowers' not in bpy.data.collections.keys():
            bone_follow_coll = bpy.data.collections.new('BoneFollowers')
            context.scene.collection.children.link(bone_follow_coll)
        else:
            bone_follow_coll = bpy.data.collections['BoneFollowers']

        if 'Targets' not in bpy.data.collections.keys():
            bone_target_coll = bpy.data.collections.new('Targets')
            context.scene.collection.children.link(bone_target_coll)
        else:
            bone_target_coll = bpy.data.collections['Targets']

        #* create empties for SRC bones.
        for bones_chain in ret_props.arma_hierarchy:
            for bone_item in bones_chain.src_bones:
                edit_bone = source_arma.data.bones[bone_item.name]
                print(f'Adding empty_box for bone {edit_bone.name}')
                empty_box = bpy.data.objects.get(edit_bone.name)
                if not empty_box:
                    empty_box = bpy.data.objects.new(edit_bone.name, None)
                    empty_box.empty_display_size = sqrt(edit_bone.length)/40
                    empty_box.empty_display_type = 'CUBE'
                    bone_follow_coll.objects.link(empty_box)

                old_rot_contr = [constr for constr in empty_box.constraints if constr.type == 'COPY_ROTATION' and constr.target == source_arma and constr.subtarget == edit_bone.name]
                if not old_rot_contr:
                    copy_rot = empty_box.constraints.new('COPY_ROTATION')
                    copy_rot.target = source_arma
                    copy_rot.subtarget = edit_bone.name

                old_loc_contr = [constr for constr in empty_box.constraints if constr.type == 'COPY_LOCATION' and constr.target == source_arma and constr.subtarget == edit_bone.name]
                if not old_loc_contr:
                    copy_loc = empty_box.constraints.new('COPY_LOCATION')
                    copy_loc.target = source_arma
                    copy_loc.subtarget = edit_bone.name

                object_name = edit_bone.name + 'T'
                empty_child = bpy.data.objects.get(object_name)
                if not empty_child:
                    empty_child = bpy.data.objects.new(object_name, None)
                    empty_child.empty_display_size = sqrt(edit_bone.length)/40
                    empty_child.empty_display_type = 'SPHERE'

                if empty_child.name not in bone_target_coll.objects.keys():
                    bone_target_coll.objects.link(empty_child)
                empty_child.parent = empty_box


        for bones_chain in ret_props.arma_hierarchy:
            if len(bones_chain.src_bones) == 0 or len(bones_chain.target_bones) == 0:
                self.report({'WARNING'}, f'Empty chain {bones_chain.name}.Skipping')
                continue

            src_bones = [b for b in bones_chain.src_bones if b.enabled]
            target_bones = [b for b in bones_chain.target_bones if b.enabled]
            if len(src_bones) != len(target_bones):
                self.report({'WARNING'}, f'Hierarchy length mismatch for {bones_chain.name} chain')
            for src_bone, target_bone in zip(src_bones, target_bones):
                # s_bone = source_arma.data.bones[src_bone.name]
                # t_bone = target_arma.data.bones[target_bone.name]
                #* set constraints on target armature bones to copy loc, rot - from target empties
                self.setup_constraints(context, source_arma.pose.bones[src_bone.name], target_arma.pose.bones[target_bone.name], target_bone.copy_loc, target_bone.copy_rot)  # first for chain roots

        return {"FINISHED"}


class ARMATURE_UL_target_chains_list(bpy.types.UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname):
        hierarchy_structure = data
        retarget_info = item
        ret_props = context.scene.retarget_settings
        arma_obj = bpy.data.objects.get(ret_props.target_armature)
        # draw_item must handle the three layout types... Usually 'DEFAULT' and 'COMPACT' can share the same code.
        if self.layout_type in {'DEFAULT', 'COMPACT'}:
            if retarget_info:
                row = layout.row(align=True)
                if arma_obj:
                    # row.prop_search(bone, "name", arma_obj.data, "bones", text='')
                    row.prop(retarget_info,'name', emboss=False, text='')
                    ic = 'CHECKBOX_HLT' if retarget_info.enabled else 'CHECKBOX_DEHLT'
                    row.prop(retarget_info, "enabled", emboss=False, icon=ic, icon_only=True)
                else:
                    row.prop(retarget_info,'name', text='', emboss=False)
                row.prop(retarget_info, "copy_rot", emboss=True, icon='CON_ROTLIKE', icon_only=True)
                row.prop(retarget_info, "copy_loc", emboss=True, icon='CON_LOCLIKE', icon_only=True)
            else:
                layout.label(text="", translate=False)
        elif self.layout_type in {'GRID'}:
            layout.alignment = 'CENTER'
            layout.label(text="")


class ARMATURE_UL_src_chains_list(bpy.types.UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname):
        hierarchy_structure = data
        ret_info = item
        ret_props = context.scene.retarget_settings
        arma_obj = bpy.data.objects.get(ret_props.src_armature)
        if self.layout_type in {'DEFAULT', 'COMPACT'}:
            if ret_info:
                row = layout.row(align=True)
                if arma_obj:
                    # row.prop_search(bone, "name", arma_obj.data, "bones", text='')
                    row.prop(ret_info, 'name', emboss=False, text='')
                    ic = 'CHECKBOX_HLT' if ret_info.enabled else 'CHECKBOX_DEHLT'
                    row.prop(ret_info, "enabled", emboss=False, icon=ic, icon_only=True)
                else:
                    row.prop(ret_info, 'name', text='', emboss=False)
            else:
                layout.label(text="", translate=False)
        elif self.layout_type in {'GRID'}:
            layout.alignment = 'CENTER'
            layout.label(text="")


class ARMATURE_PT_BonesHierarchy(bpy.types.Panel):
    bl_idname = 'ARMATURE_PT_BonesHierarchy'
    bl_label = 'Retargeting Hierarchy'
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Tools'

    def draw(self, context):
        ret_props = context.scene.retarget_settings
        layout = self.layout
        layout.prop_search(ret_props, 'src_armature', bpy.data, 'objects')
        layout.prop_search(ret_props, 'target_armature', bpy.data, 'objects')

        layout.operator('object.build_bones_hierarchy')

        row = layout.row()
        row.template_list("UI_UL_list", "name", ret_props, "arma_hierarchy", ret_props, "hierarchy_idx")
        col = row.column(align=True)
        col.operator("object.add_chain", icon='ADD', text="")
        col.operator("object.remove_chain", icon='REMOVE', text="")

        row = layout.row(align=True)
        split = row.split(factor=0.6, align=True)
        sub_col = split.column(align=True)
        sub_row = sub_col.row(align=True)
        sub_row.label(text='Target Bones')
        sub_row.operator("object.add_chain_bone", icon='ADD', text="").do_src = False
        sub_row.operator("object.remove_chain_bone", icon='REMOVE', text="").do_src = False
        if ret_props.hierarchy_idx < len(ret_props.arma_hierarchy):
            hierarchy_props = ret_props.arma_hierarchy[ret_props.hierarchy_idx]
            sub_col.template_list("ARMATURE_UL_target_chains_list", "", hierarchy_props, "target_bones", hierarchy_props, "target_bone_idx")

            split = split.split()
            sub_col = split.column(align=True)
            sub_row = sub_col.row(align=True)
            sub_row.label(text='Source Bones')
            sub_row.operator("object.add_chain_bone", icon='ADD', text="").do_src = True
            sub_row.operator("object.remove_chain_bone", icon='REMOVE', text="").do_src = True
            src_active_bones_chain = ret_props.arma_hierarchy[ret_props.hierarchy_idx]
            sub_col.template_list("ARMATURE_UL_src_chains_list", "", hierarchy_props, "src_bones", hierarchy_props, "src_bone_idx")

        layout.operator('object.clean_constraints')
        layout.operator('object.retarget_using_empties')

        # save and raad json opers
        col = layout.column(align=True)
        col.operator('object.write_chain')
        col.operator('object.read_chain')


# operator that will write src and target armature and their mathcing bones to json
class RET_OT_WriteChain(bpy.types.Operator):
    bl_idname = "object.write_chain"
    bl_label = "Write To File"
    bl_description = "Write Rig chain relations to file (json)"
    bl_options = {"REGISTER", "UNDO"}

    directory: bpy.props.StringProperty(subtype="DIR_PATH")
    filename: bpy.props.StringProperty(name="File Name", default="file.json")
    filepath: bpy.props.StringProperty(subtype="FILE_PATH")

    def execute(self, context):
        full_path = bpy.path.abspath(self.directory + self.filename)
        self.json_write(full_path)
        self.report({'INFO'}, "File saved to: " + full_path)
        return {'FINISHED'}

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

    @staticmethod
    def json_write(filepath):
        scene = bpy.context.scene
        retarget_settings = scene.retarget_settings

        data = {
            'src_armature': retarget_settings.src_armature,
            'target_armature': retarget_settings.target_armature,
            'arma_hierarchy': []
        }

        for hierarchy in retarget_settings.arma_hierarchy:
            hierarchy_data = {
                'name': hierarchy.name,
                'src_bones': [],
                'target_bones': [],
                'src_bone_idx': hierarchy.src_bone_idx,
                'target_bone_idx': hierarchy.target_bone_idx
            }

            for src_bone in hierarchy.src_bones:
                hierarchy_data['src_bones'].append({
                    'name': src_bone.name,
                    'enabled': src_bone.enabled,
                    'copy_rot': src_bone.copy_rot,
                    'copy_loc': src_bone.copy_loc
                })

            for target_bone in hierarchy.target_bones:
                hierarchy_data['target_bones'].append({
                    'name': target_bone.name,
                    'enabled': target_bone.enabled,
                    'copy_rot': target_bone.copy_rot,
                    'copy_loc': target_bone.copy_loc
                })

            data['arma_hierarchy'].append(hierarchy_data)

        with open(filepath, 'w') as f:
            json.dump(data, f, indent=4)


# json read oper
class RET_OT_ReadChain(bpy.types.Operator):
    bl_idname = "object.read_chain"
    bl_label = "Read retargeting info from file"
    bl_description = "Read retargeting info from file (json)"
    bl_options = {"REGISTER", "UNDO"}

    filepath: bpy.props.StringProperty(subtype="FILE_PATH")

    def execute(self, context):
        self.report({'INFO'}, "Selected file: " + self.filepath)
        self.json_read(self.filepath)
        return {'FINISHED'}

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}


    @staticmethod
    def json_read(filepath):
        scene = bpy.context.scene
        retarget_settings = scene.retarget_settings

        with open(filepath, 'r') as f:
            data = json.load(f)

        retarget_settings.src_armature = data['src_armature']
        retarget_settings.target_armature = data['target_armature']
        retarget_settings.arma_hierarchy.clear()

        for hierarchy_data in data['arma_hierarchy']:
            hierarchy = retarget_settings.arma_hierarchy.add()
            hierarchy.name = hierarchy_data['name']
            hierarchy.src_bone_idx = hierarchy_data['src_bone_idx']
            hierarchy.target_bone_idx = hierarchy_data['target_bone_idx']

            for src_bone_data in hierarchy_data['src_bones']:
                src_bone = hierarchy.src_bones.add()
                src_bone.name = src_bone_data['name']
                src_bone.enabled = src_bone_data['enabled']
                src_bone.copy_rot = src_bone_data['copy_rot']
                src_bone.copy_loc = src_bone_data['copy_loc']

            for target_bone_data in hierarchy_data['target_bones']:
                target_bone = hierarchy.target_bones.add()
                target_bone.name = target_bone_data['name']
                target_bone.enabled = target_bone_data['enabled']
                target_bone.copy_rot = target_bone_data['copy_rot']
                target_bone.copy_loc = target_bone_data['copy_loc']




class ChainBones(bpy.types.PropertyGroup):
    name: bpy.props.StringProperty()
    enabled: bpy.props.BoolProperty(name='Enabled', description='', default=True)
    copy_rot: bpy.props.BoolProperty(name='Copy Rot', description='', default=True)
    copy_loc: bpy.props.BoolProperty(name='Copy Loc', description='', default=False)


class RET_OT_AddChainBone(bpy.types.Operator):
    bl_idname = "object.add_chain_bone"
    bl_label = "Add Chain Bone"
    bl_description = "Add Chain Bone"
    bl_options = {"REGISTER", "UNDO"}

    do_src: bpy.props.BoolProperty(name='Source bones?', default=True)
    name: bpy.props.StringProperty(name='Name', description='', default='Bone')

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)

    def draw(self, context):
        layout = self.layout
        ret_props = context.scene.retarget_settings
        arma_obj = bpy.data.objects.get(ret_props.src_armature) if self.do_src else bpy.data.objects.get(ret_props.target_armature)
        if arma_obj:
            layout.prop_search(self, "name", arma_obj.data, "bones")
        else:
            arma_type = 'source' if self.do_src else 'target'
            layout.label(text=f'No {arma_type} armature to pick bone from')

    def execute(self, context):
        ret_props = context.scene.retarget_settings
        if self.name:
            hierarchy = ret_props.arma_hierarchy[ret_props.hierarchy_idx]
            if self.do_src:
                new_bone = hierarchy.src_bones.add()
            else:
                new_bone = hierarchy.target_bones.add()
            new_bone.name = self.name
        else:
            self.report({'WARNING'}, 'Provide bone name')

        return {"FINISHED"}


class RET_OT_RemoveChainBone(bpy.types.Operator):
    bl_idname = "object.remove_chain_bone"
    bl_label = "Remove Bone"
    bl_description = "Remove Chain Bone"
    bl_options = {"REGISTER", "UNDO"}

    do_src: bpy.props.BoolProperty(name='Source bones?', default=True)

    def execute(self, context):
        ret_props = context.scene.retarget_settings
        hierarchy = ret_props.arma_hierarchy[ret_props.hierarchy_idx]
        if self.do_src:
            hierarchy.src_bones.remove(hierarchy.src_bone_idx)
        else:
            hierarchy.target_bones.remove(hierarchy.target_bone_idx)
        return {"FINISHED"}


class ArmaHierarchyStructures(bpy.types.PropertyGroup):
    name: bpy.props.StringProperty()

    src_bones: bpy.props.CollectionProperty(type=ChainBones)
    target_bone_idx: bpy.props.IntProperty(name='Active Hierarchy Idx', description='', default= 1, min=0, max=100)

    target_bones: bpy.props.CollectionProperty(type=ChainBones)
    src_bone_idx: bpy.props.IntProperty(name='Active Hierarchy Idx', description='', default= 1, min=0, max=100)


class RET_OT_AddChain(bpy.types.Operator):
    bl_idname = "object.add_chain"
    bl_label = "Add Chain"
    bl_description = "Add Chain"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        chain = context.scene.retarget_settings.arma_hierarchy.add()
        return {"FINISHED"}

class RET_OT_RemoveChain(bpy.types.Operator):
    bl_idname = "object.remove_chain"
    bl_label = "Remove Chain"
    bl_description = "Remove Chain"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        ret_props = context.scene.retarget_settings
        ret_props.arma_hierarchy.remove(ret_props.hierarchy_idx)
        return {"FINISHED"}

class RetargetingSettings(bpy.types.PropertyGroup):
    src_armature: bpy.props.StringProperty(name='Source Rig')
    target_armature: bpy.props.StringProperty(name='Target Rig')
    arma_hierarchy: bpy.props.CollectionProperty(type=ArmaHierarchyStructures)
    hierarchy_idx: bpy.props.IntProperty(name='Active Hierarchy Idx', description='', default= 1, min=0, max=100)


classes = (
    RET_OT_RetargetByEmpties,
    RET_OT_BuildBonesHierarchy,
    RET_OT_CleanConstraintsHierarchy,
    RET_OT_WriteChain,
    RET_OT_ReadChain,
    ChainBones,
    ArmaHierarchyStructures,
    RetargetingSettings,
    RET_OT_AddChainBone,
    RET_OT_RemoveChainBone,
    RET_OT_AddChain,
    RET_OT_RemoveChain,
    ARMATURE_PT_BonesHierarchy,
    ARMATURE_UL_src_chains_list,
    ARMATURE_UL_target_chains_list,
)

def register():
    from bpy.utils import register_class
    for cls in classes:
        register_class(cls)

    bpy.types.Scene.retarget_settings = bpy.props.PointerProperty(type=RetargetingSettings)

def unregister():
    from bpy.utils import unregister_class
    for cls in reversed(classes):
        unregister_class(cls)

    del bpy.types.Scene.retarget_settings



