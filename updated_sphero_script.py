import bpy
import os
import math
import random
from mathutils import Vector
import csv

import platform
if platform.system() == "Windows":
    output_dir = os.path.join(os.path.expanduser("~"), "Desktop", "SpheroRender")
else:
    output_dir = os.path.join(os.path.expanduser("~/Desktop/SpheroRender"))

# Create output directory if it doesn't exist
os.makedirs(output_dir, exist_ok=True)

if bpy.app.version < (2, 83, 0):
    print("Warning: This script was developed for Blender 2.83+")

scene = bpy.context.scene
scene.render.fps = 60
scene.frame_start = 1
scene.frame_end = 180  # Extended animation length for full impact and rebound

# Set up physics timestep for more accurate simulation
scene.rigidbody_world.time_scale = 1.0
scene.rigidbody_world.steps_per_second = 240  # Increased for better accuracy
scene.rigidbody_world.solver_iterations = 20  # Increased for more stable collisions

bpy.context.scene.frame_set(scene.frame_start)

# Clear existing objects
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete()

# Set up rendering
scene = bpy.context.scene
scene.render.engine = 'BLENDER_EEVEE_NEXT'  # Changed to Eevee
scene.eevee.taa_samples = 64       # Temporal Anti-Aliasing samples
scene.render.resolution_x = 1920
scene.render.resolution_y = 1080
scene.render.fps = 60


# Create ground plane
bpy.ops.mesh.primitive_plane_add(size=50, location=(0, 0, 0))
ground = bpy.context.active_object
ground.name = "Ground"

# Add material to ground
mat = bpy.data.materials.new(name="GroundMaterial")
mat.use_nodes = True
nodes = mat.node_tree.nodes
bsdf = nodes.get("Principled BSDF")
bsdf.inputs["Base Color"].default_value = (0.2, 0.2, 0.2, 1.0)
ground.data.materials.append(mat)

# Create wall
bpy.ops.mesh.primitive_cube_add(size=1, location=(0, 13, 0.25))
wall = bpy.context.active_object
wall.name = "Wall"
wall.scale = (20, 0.5, 4)
wall.location = (0, 15, 2)

# Add material to wall
mat = bpy.data.materials.new(name="WallMaterial")
mat.use_nodes = True
nodes = mat.node_tree.nodes
bsdf = nodes.get("Principled BSDF")
bsdf.inputs["Base Color"].default_value = (0.8, 0.8, 0.8, 1.0)
wall.data.materials.append(mat)

# Create energy absorption barrier
def create_barrier():
    # Base structure
    bpy.ops.mesh.primitive_cube_add(size=1, location=(0, 13, 1))
    base = bpy.context.active_object
    base.name = "BarrierBase"
    base.scale = (15, 1, 0.5)

    # Add material to base
    mat = bpy.data.materials.new(name="BaseMaterial")
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    bsdf = nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = (0.5, 0.3, 0.1, 1.0)
    base.data.materials.append(mat)

    # High-density foam backing
    bpy.ops.mesh.primitive_cube_add(size=1, location=(0, 13.6, 0.85))
    backing = bpy.context.active_object
    backing.name = "FoamBacking"
    backing.scale = (14, 0.5, 1.2)
    backing.location = (0, 13.6, 0.85)

    # Add material to backing
    mat = bpy.data.materials.new(name="BackingMaterial")
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    bsdf = nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = (0.2, 0.4, 0.6, 1.0)
    bsdf.inputs["Roughness"].default_value = 0.9
    backing.data.materials.append(mat)

    # Springs array
    springs = []
    num_springs = 8
    spring_width = 12 / num_springs

    for i in range(num_springs):
        x_pos = -6 + i * spring_width + spring_width/2
        bpy.ops.mesh.primitive_cylinder_add(
            vertices=16,
            radius=0.2,
            depth=0.8,
            location=(x_pos, 14.1, 1.6)
        )
        spring = bpy.context.active_object
        spring.name = f"Spring_{i}"

        # Add spring material
        mat = bpy.data.materials.new(name=f"SpringMaterial_{i}")
        mat.use_nodes = True
        nodes = mat.node_tree.nodes
        bsdf = nodes.get("Principled BSDF")
        bsdf.inputs["Base Color"].default_value = (0.7, 0.7, 0.7, 1.0)
        bsdf.inputs["Metallic"].default_value = 0.9
        spring.data.materials.append(mat)

        # Create spring coil geometry
        bpy.ops.curve.primitive_bezier_circle_add(radius=0.2, location=(x_pos, 14.1, 1.6))
        circle = bpy.context.active_object

        bpy.ops.curve.primitive_bezier_curve_add(location=(x_pos, 14.1, 1.6))
        curve = bpy.context.active_object
        curve.data.dimensions = '3D'
        curve.data.resolution_u = 64
        curve.data.bevel_object = circle

        # Create spring coil shape
        bezier_points = curve.data.splines[0].bezier_points
        bezier_points.add(10-1)  # Add points (minus the one that's already there)

        coil_height = 0.8
        coil_radius = 0.2
        coil_turns = 4

        for j in range(len(bezier_points)):
            t = j / (len(bezier_points) - 1)
            angle = t * coil_turns * 2 * math.pi
            z = t * coil_height
            x = math.cos(angle) * coil_radius
            y = math.sin(angle) * coil_radius

            bezier_points[j].co = (x, y, z)
            bezier_points[j].handle_left_type = 'AUTO'
            bezier_points[j].handle_right_type = 'AUTO'

        # Apply material to spring coil
        curve.data.materials.append(mat)
        springs.append(curve)

        # Remove the cylinder (was just for placement reference)
        bpy.data.objects.remove(spring)
        bpy.data.objects.remove(circle)

    # Impact surface (curved)
    bpy.ops.mesh.primitive_cube_add(size=1, location=(0, 14.7, 1.6))
    impact = bpy.context.active_object
    impact.name = "ImpactSurface"
    impact.scale = (14, 0.3, 1.2)

    # Subdivide the impact surface for better deformation
    bpy.context.view_layer.objects.active = impact
    bpy.ops.object.modifier_add(type='SUBSURF')
    impact.modifiers["Subdivision"].levels = 3
    impact.modifiers["Subdivision"].render_levels = 3
    
    # Apply the subdivision modifier
    bpy.ops.object.modifier_apply(modifier="Subdivision")
    
    # Add a simple deform modifier for better bending
    bpy.ops.object.modifier_add(type='SIMPLE_DEFORM')
    impact.modifiers["SimpleDeform"].deform_method = 'BEND'
    impact.modifiers["SimpleDeform"].deform_axis = 'Y'
    impact.modifiers["SimpleDeform"].angle = -0.5
    
    # Apply the bend modifier
    bpy.ops.object.modifier_apply(modifier="SimpleDeform")

    # Add material to impact surface
    mat = bpy.data.materials.new(name="ImpactMaterial")
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    bsdf = nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = (0.1, 0.2, 0.7, 1.0)
    bsdf.inputs["Roughness"].default_value = 0.3
    impact.data.materials.append(mat)

    return base, backing, springs, impact

barrier_base, barrier_backing, barrier_springs, barrier_impact = create_barrier()

# Create Sphero Bolt
bpy.ops.mesh.primitive_uv_sphere_add(radius=0.75, location=(-10, 5, 0.75))  # Start further back
sphero = bpy.context.active_object
sphero.name = "Sphero"

# Add material to Sphero
mat = bpy.data.materials.new(name="SpheroMaterial")
mat.use_nodes = True
nodes = mat.node_tree.nodes

# Try to get the Principled BSDF node
bsdf = nodes.get("Principled BSDF")

if bsdf:
    # Set properties on the BSDF if it exists
    bsdf.inputs["Base Color"].default_value = (0.9, 0.9, 0.9, 1.0)
    bsdf.inputs["Metallic"].default_value = 0.2
    bsdf.inputs["Roughness"].default_value = 0.2

    # Check if 'Clearcoat' exists before trying to access it
    if "Clearcoat" in bsdf.inputs:
        bsdf.inputs["Clearcoat"].default_value = 1.0
    else:
        print("Clearcoat input not available in the Principled BSDF node.")
else:
    print("Principled BSDF node not found.")

# Apply the material to the Sphero object
sphero.data.materials.append(mat)


# Set up lighting
def create_lighting():
    # Key light
    bpy.ops.object.light_add(type='AREA', radius=5, location=(10, -10, 15))
    key_light = bpy.context.active_object
    key_light.name = "KeyLight"
    key_light.data.energy = 500
    key_light.data.size = 5

    # Fill light
    bpy.ops.object.light_add(type='AREA', radius=5, location=(-10, -10, 10))
    fill_light = bpy.context.active_object
    fill_light.name = "FillLight"
    fill_light.data.energy = 300
    fill_light.data.size = 7

    # Rim light
    bpy.ops.object.light_add(type='AREA', radius=3, location=(0, -15, 8))
    rim_light = bpy.context.active_object
    rim_light.name = "RimLight"
    rim_light.data.energy = 400
    rim_light.data.size = 3

    # Environment lighting with HDRI
    world = bpy.context.scene.world
    if not world:
        world = bpy.data.worlds.new("World")
        bpy.context.scene.world = world

    world.use_nodes = True
    node_tree = world.node_tree

    # Clear existing nodes
    for node in node_tree.nodes:
        node_tree.nodes.remove(node)

    # Create new nodes
    node_background = node_tree.nodes.new(type='ShaderNodeBackground')
    node_environment = node_tree.nodes.new(type='ShaderNodeTexEnvironment')
    node_output = node_tree.nodes.new(type='ShaderNodeOutputWorld')

    # Connect nodes
    node_tree.links.new(node_environment.outputs["Color"], node_background.inputs["Color"])
    node_tree.links.new(node_background.outputs["Background"], node_output.inputs["Surface"])

    # Position nodes
    node_environment.location = (-300, 0)
    node_background.location = (0, 0)
    node_output.location = (300, 0)

    # Create a simple procedural environment
    mapping = node_tree.nodes.new(type='ShaderNodeMapping')
    tex_coord = node_tree.nodes.new(type='ShaderNodeTexCoord')
    gradient = node_tree.nodes.new(type='ShaderNodeTexGradient')

    node_tree.links.new(tex_coord.outputs["Generated"], mapping.inputs["Vector"])
    node_tree.links.new(mapping.outputs["Vector"], gradient.inputs["Vector"])
    node_tree.links.new(gradient.outputs["Color"], node_background.inputs["Color"])

    # Set gradient colors for a nice studio look
    color_ramp = node_tree.nodes.new(type='ShaderNodeValToRGB')
    color_ramp.color_ramp.elements[0].color = (0.05, 0.05, 0.1, 1.0)
    color_ramp.color_ramp.elements[1].color = (0.2, 0.3, 0.4, 1.0)

    node_tree.links.new(gradient.outputs["Color"], color_ramp.inputs["Fac"])
    node_tree.links.new(color_ramp.outputs["Color"], node_background.inputs["Color"])

    return key_light, fill_light, rim_light

key_light, fill_light, rim_light = create_lighting()

# Set up camera
bpy.ops.object.camera_add(location=(15, -15, 6), rotation=(math.radians(70), 0, math.radians(135)))
camera = bpy.context.active_object
camera.name = "Camera"
bpy.context.scene.camera = camera

# Track camera to look at sphero
track_to = camera.constraints.new(type='TRACK_TO')
track_to.target = sphero
track_to.track_axis = 'TRACK_NEGATIVE_Z'
track_to.up_axis = 'UP_Y'

# Add rigidbody to sphero with improved physics
bpy.context.view_layer.objects.active = sphero
bpy.ops.rigidbody.object_add()
sphero.rigid_body.mass = 2.0
sphero.rigid_body.restitution = 0.8  # Bounciness
sphero.rigid_body.friction = 0.3
sphero.rigid_body.collision_shape = 'SPHERE'
sphero.rigid_body.linear_damping = 0.1  # Reduced damping for more realistic movement
sphero.rigid_body.angular_damping = 0.1
sphero.rigid_body.use_deactivation = False  # Prevent the object from going to sleep

# Create an empty to apply force to Sphero
bpy.ops.object.empty_add(type='PLAIN_AXES', location=sphero.location)
force_empty = bpy.context.active_object
force_empty.name = "ForceEmpty"
force_empty.parent = sphero

# Add constraint to keep force_empty at the same location as sphero
copy_loc = force_empty.constraints.new(type='COPY_LOCATION')
copy_loc.target = sphero

# Add a force field for initial acceleration
bpy.ops.object.effector_add(type='FORCE', location=sphero.location)
push_force = bpy.context.active_object
push_force.name = "SpheroPushForce"
push_force.field.strength = 30  # Increased strength
push_force.field.flow = 1.0  # Ensure force flow is in the right direction
push_force.field.falloff_type = 'SPHERE'
push_force.field.use_max_distance = True
push_force.field.distance_max = 15

# Position force in the direction of barrier
push_force.location = (-10, 5, 0.75)  # Same as sphero's starting position
push_force.rotation_euler = (0, 0, math.radians(-45))  # Point towards barrier

# Animate push force to control acceleration
push_force.keyframe_insert(data_path="field.strength", frame=1)
push_force.field.strength = 30  # Start strong
push_force.keyframe_insert(data_path="field.strength", frame=20)
push_force.field.strength = 15  # Gradual reduction
push_force.keyframe_insert(data_path="field.strength", frame=40)
push_force.field.strength = 0  # Stop pushing - let momentum carry forward
push_force.keyframe_insert(data_path="field.strength", frame=60)

# Add rigidbody to wall
bpy.context.view_layer.objects.active = wall
bpy.ops.rigidbody.object_add()
wall.rigid_body.type = 'PASSIVE'
wall.rigid_body.collision_shape = 'BOX'

# Add rigidbody to ground
bpy.context.view_layer.objects.active = ground
bpy.ops.rigidbody.object_add()
ground.rigid_body.type = 'PASSIVE'
ground.rigid_body.collision_shape = 'BOX'

# Add rigidbody to barrier parts
bpy.context.view_layer.objects.active = barrier_base
bpy.ops.rigidbody.object_add()
barrier_base.rigid_body.type = 'PASSIVE'
barrier_base.rigid_body.collision_shape = 'BOX'

# Setup spring system - backing with active physics
bpy.context.view_layer.objects.active = barrier_backing
bpy.ops.rigidbody.object_add()
barrier_backing.rigid_body.type = 'ACTIVE'
barrier_backing.rigid_body.mass = 3.0  # Increased mass for stability
barrier_backing.rigid_body.collision_shape = 'BOX'
barrier_backing.rigid_body.kinematic = True  # Will be animated with compression

# Impact surface with deformation
bpy.context.view_layer.objects.active = barrier_impact
bpy.ops.rigidbody.object_add()
barrier_impact.rigid_body.type = 'ACTIVE'
barrier_impact.rigid_body.mass = 1.0  # Lighter for more responsiveness
barrier_impact.rigid_body.collision_shape = 'CONVEX_HULL'
barrier_impact.rigid_body.kinematic = True  # Will be animated with deformation

# Estimated impact frame (will be recorded during simulation)
estimated_impact_frame = 90

# Add animation for the barrier compression with improved physics
def animate_barrier_compression(impact_frame):
    # Store original positions
    original_impact_pos = barrier_impact.location.copy()
    original_backing_pos = barrier_backing.location.copy()
    
    # Initial position keyframes
    barrier_impact.keyframe_insert(data_path="location", frame=1)
    barrier_backing.keyframe_insert(data_path="location", frame=1)
    
    # Pre-impact anticipation (slight movement forward)
    pre_impact = impact_frame - 3
    barrier_impact.location.y += 0.05
    barrier_impact.keyframe_insert(data_path="location", frame=pre_impact)
    
    # Maximum compression at impact
    max_compression_frame = impact_frame + 5
    compression_amount = 0.8  # Increased compression
    
    # Compressed position (impact phase)
    barrier_impact.location.y -= compression_amount  # Significant compression
    barrier_impact.keyframe_insert(data_path="location", frame=max_compression_frame)
    
    barrier_backing.location.y -= compression_amount * 0.7  # Proportional compression
    barrier_backing.keyframe_insert(data_path="location", frame=max_compression_frame)
    
    # Spring release - overshoot forward (springs pushing back)
    release_frame = max_compression_frame + 8
    barrier_impact.location.y += compression_amount * 1.3  # Overshoot to show elastic force
    barrier_impact.keyframe_insert(data_path="location", frame=release_frame)
    
    barrier_backing.location.y += compression_amount * 0.9
    barrier_backing.keyframe_insert(data_path="location", frame=release_frame)
    
    # Secondary oscillation - backward
    oscillation1_frame = release_frame + 6
    barrier_impact.location.y -= compression_amount * 0.4
    barrier_impact.keyframe_insert(data_path="location", frame=oscillation1_frame)
    
    barrier_backing.location.y -= compression_amount * 0.3
    barrier_backing.keyframe_insert(data_path="location", frame=oscillation1_frame)
    
    # Final oscillation - forward
    oscillation2_frame = oscillation1_frame + 5
    barrier_impact.location.y += compression_amount * 0.2
    barrier_impact.keyframe_insert(data_path="location", frame=oscillation2_frame)
    
    barrier_backing.location.y += compression_amount * 0.15
    barrier_backing.keyframe_insert(data_path="location", frame=oscillation2_frame)
    
    # Return to resting state (slightly offset from original)
    settle_frame = oscillation2_frame + 8
    
    # Calculate final resting positions
    final_impact_y = original_impact_pos.y - compression_amount * 0.05  # Permanent slight deformation
    final_backing_y = original_backing_pos.y - compression_amount * 0.03
    
    barrier_impact.location.y = final_impact_y
    barrier_impact.keyframe_insert(data_path="location", frame=settle_frame)
    
    barrier_backing.location.y = final_backing_y
    barrier_backing.keyframe_insert(data_path="location", frame=settle_frame)
    
    # Add easing to make the animation more realistic
    for obj in [barrier_impact, barrier_backing]:
        if obj.animation_data and obj.animation_data.action:
            for fcurve in obj.animation_data.action.fcurves:
                for kf in fcurve.keyframe_points:
                    kf.interpolation = 'BEZIER'
                    kf.handle_left_type = 'AUTO_CLAMPED'
                    kf.handle_right_type = 'AUTO_CLAMPED'

    # Add deformation to the impact surface during compression
    # Create shape keys for deformation
    bpy.context.view_layer.objects.active = barrier_impact
    barrier_impact.shape_key_add(name="Basis")
    
    # Create impact deformation shape key
    impact_key = barrier_impact.shape_key_add(name="Impact")
    # Select impact key
    barrier_impact.active_shape_key_index = 1
    
    # Deform vertices to create concave impact point
    impact_depth = 0.4
    for vert in barrier_impact.data.vertices:
        # Calculate distance from center
        local_co = vert.co.copy()
        dist_from_center = math.sqrt((local_co.x)**2)
        
        # Create parabolic deformation (deeper in center, less on edges)
        if dist_from_center < 5:  # Limit deformation to center area
            deform_factor = 1 - (dist_from_center / 5)**2
            impact_key.data[vert.index].co.y -= impact_depth * deform_factor
    
    # Animate shape key
    impact_key.value = 0
    impact_key.keyframe_insert(data_path="value", frame=pre_impact)
    impact_key.value = 1
    impact_key.keyframe_insert(data_path="value", frame=max_compression_frame)
    impact_key.value = 0.1
    impact_key.keyframe_insert(data_path="value", frame=release_frame)
    impact_key.value = 0.4
    impact_key.keyframe_insert(data_path="value", frame=oscillation1_frame)
    impact_key.value = 0.15
    impact_key.keyframe_insert(data_path="value", frame=settle_frame)
    
    # Add easing to shape key animation
    if barrier_impact.data.shape_keys and barrier_impact.data.shape_keys.animation_data:
        for fcurve in barrier_impact.data.shape_keys.animation_data.action.fcurves:
            for kf in fcurve.keyframe_points:
                kf.interpolation = 'BEZIER'
                kf.handle_left_type = 'AUTO_CLAMPED'
                kf.handle_right_type = 'AUTO_CLAMPED'

# We'll call this function later with the actual impact frame detected during simulation

# Add rotation to Sphero as it rolls
def animate_sphero_rotation():
    # Create an empty to control Sphero's rotation
    bpy.ops.object.empty_add(type='PLAIN_AXES', location=sphero.location)
    rotation_control = bpy.context.active_object
    rotation_control.name = "SpheroRotationControl"
    
    # Parent the sphero to the rotation control
    sphero.parent = rotation_control
    
    # Animate the empty's rotation
    rotation_control.keyframe_insert(data_path="rotation_euler", frame=1)
    
    # Full rotation around X axis (rolling forward)
    rotation_control.rotation_euler.x = 2 * math.pi * 6  # 6 complete rotations
    rotation_control.keyframe_insert(data_path="rotation_euler", frame=estimated_impact_frame)
    
    # Add some Y rotation to simulate slight directional changes
    rotation_control.rotation_euler.y = 0
    rotation_control.keyframe_insert(data_path="rotation_euler", frame=1)
    rotation_control.rotation_euler.y = 0.2
    rotation_control.keyframe_insert(data_path="rotation_euler", frame=30)
    rotation_control.rotation_euler.y = -0.1
    rotation_control.keyframe_insert(data_path="rotation_euler", frame=60)
    rotation_control.rotation_euler.y = 0
    rotation_control.keyframe_insert(data_path="rotation_euler", frame=estimated_impact_frame)
    
    # Make rotation curves smoother
    for fcurve in rotation_control.animation_data.action.fcurves:
        for kf in fcurve.keyframe_points:
            kf.interpolation = 'BEZIER'
            kf.handle_left_type = 'AUTO_CLAMPED'
            kf.handle_right_type = 'AUTO_CLAMPED'

# Animate the camera to follow the action
def animate_camera():
    # Initial position - wider view to see full acceleration
    camera.keyframe_insert(data_path="location", frame=1)
    
    # Follow shot as Sphero accelerates
    camera.location = (10, -5, 5)
    camera.keyframe_insert(data_path="location", frame=40)
    
    # Move to side angle to see impact
    camera.location = (8, 0, 3)
    camera.keyframe_insert(data_path="location", frame=estimated_impact_frame - 10)
    
    # Close-up for impact moment
    camera.location = (5, 2, 3)
    camera.keyframe_insert(data_path="location", frame=estimated_impact_frame)
    
    # Pull back to see rebound
    camera.location = (5, -5, 4)
    camera.keyframe_insert(data_path="location", frame=estimated_impact_frame + 20)
    
    # Final wide view
    camera.location = (8, -8, 6)
    camera.keyframe_insert(data_path="location", frame=scene.frame_end)
    
    # Smooth camera movement
    for fcurve in camera.animation_data.action.fcurves:
        for kf in fcurve.keyframe_points:
            kf.interpolation = 'BEZIER'
            kf.handle_left_type = 'AUTO_CLAMPED'
            kf.handle_right_type = 'AUTO_CLAMPED'

# Add a motion blur effect for realism
scene.render.use_motion_blur = True
scene.render.motion_blur_shutter = 0.5

# Add force to create a realistic rebound effect
def add_rebound_force():
    # Create an empty that will emit the force at the impact point
    bpy.ops.object.empty_add(type='PLAIN_AXES', location=(0, 13.8, 0.75))
    rebound_empty = bpy.context.active_object
    rebound_empty.name = "ReboundEmpty"
    
    # Add force field to the empty
    bpy.ops.object.effector_add(type='FORCE', location=rebound_empty.location)
    rebound_force = bpy.context.active_object
    rebound_force.name = "ReboundForce"
    rebound_force.parent = rebound_empty
    
    # Configure the force field
    rebound_force.field.strength = 0  # Start with no force
    rebound_force.field.falloff_type = 'SPHERE'
    rebound_force.field.falloff_power = 2  # Quadratic falloff for more realistic force
    rebound_force.field.use_max_distance = True
    rebound_force.field.distance_max = 5  # Increased range of effect
    
    # Animate the force field - active only during impact and rebound
    rebound_force.keyframe_insert(data_path="field.strength", frame=estimated_impact_frame - 5)
    
    # Build up force before visible impact
    rebound_force.field.strength = -20  # Start building repulsion
    rebound_force.keyframe_insert(data_path="field.strength", frame=estimated_impact_frame)
    
    # Maximum repulsion at compression peak
    rebound_force.field.strength = -100  # Strong repulsion force
    rebound_force.keyframe_insert(data_path="field.strength", frame=estimated_impact_frame + 5)
    
    # Quickly fade out force
    reboun