import bpy
import os
import math
import csv
from mathutils import Vector

# Configuration
scene = bpy.context.scene
fps = scene.render.fps
dt = 1.0 / fps  # Time step between frames
mass = 0.5  # kg (mass of Sphero)
g = 9.81  # m/s²
base_path = "/content/drive/MyDrive/SpheroRender"
log_file = os.path.join(base_path, "sphero_physics_log.csv")

# Ensure the output directory exists
os.makedirs(base_path, exist_ok=True)

# Set up the simulation with existing code...
# [EXISTING SETUP CODE REMAINS THE SAME]

# Add this function at the end of your script
def collect_physics_data():
    """
    Enhanced physics data collector that runs AFTER the simulation is baked
    This ensures the rigid body simulation data is available
    """
    print("Starting physics data collection...")
    
    # Get the Sphero object
    sphero = bpy.data.objects.get("Sphero")
    if not sphero:
        print("Error: Sphero object not found!")
        return
    
    # Make sure the simulation is baked before we start collecting data
    print("Ensuring physics cache is baked...")
    bpy.ops.ptcache.bake_all(bake=True)
    
    # Open CSV file for writing
    with open(log_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'Frame', 'Time(s)', 
            'Position_X', 'Position_Y', 'Position_Z',
            'Velocity_X', 'Velocity_Y', 'Velocity_Z', 'Speed',
            'Acceleration_X', 'Acceleration_Y', 'Acceleration_Z', 'Acceleration_Magnitude',
            'Force_X', 'Force_Y', 'Force_Z', 'Force_Magnitude',
            'Momentum_X', 'Momentum_Y', 'Momentum_Z', 'Momentum_Magnitude',
            'Kinetic_Energy', 'Potential_Energy', 'Total_Energy',
            'Event'
        ])
        
        # Variables to track simulation state
        start_pos = None
        prev_pos = None
        prev_vel = None
        prev_accel = None
        prev_dir_y = None
        impact_detected = False
        impact_frame = None
        
        # Process all frames
        for frame in range(scene.frame_start, scene.frame_end + 1):
            # Set the current frame and update the scene
            scene.frame_set(frame)
            bpy.context.view_layer.update()
            
            # Calculate time
            time = (frame - scene.frame_start) * dt
            
            # Record initial position if first frame
            if start_pos is None:
                start_pos = sphero.location.copy()
            
            # Current position
            curr_pos = sphero.location.copy()
            
            # Calculate velocity
            if prev_pos is not None:
                velocity = (curr_pos - prev_pos) / dt
            else:
                velocity = Vector((0, 0, 0))
            
            # Calculate speed
            speed = velocity.length
            
            # Calculate acceleration
            if prev_vel is not None:
                acceleration = (velocity - prev_vel) / dt
            else:
                acceleration = Vector((0, 0, 0))
            
            # Calculate acceleration magnitude
            accel_magnitude = acceleration.length
            
            # Force (F = ma)
            force = acceleration * mass
            force_magnitude = force.length
            
            # Momentum (p = mv)
            momentum = velocity * mass
            momentum_magnitude = momentum.length
            
            # Energy calculations
            # Kinetic energy: 1/2 * m * v²
            KE = 0.5 * mass * speed ** 2
            
            # Potential energy: m * g * h (relative to starting position)
            height = curr_pos.z - start_pos.z
            PE = mass * g * height
            
            # Total energy
            total_energy = KE + PE
            
            # Event detection
            event = ""
            
            # Detect direction change (impact)
            curr_dir_y = 1 if velocity.y > 0 else -1
            if prev_dir_y is not None and curr_dir_y != prev_dir_y:
                if velocity.length > 0.1:  # Ignore very small movements
                    if not impact_detected or (impact_detected and frame > impact_frame + 10):
                        event = "DIRECTION_CHANGE"
                        impact_detected = True
                        impact_frame = frame
            
            # Detect sudden acceleration (impact)
            if accel_magnitude > 30 and not impact_detected:  # Threshold based on experimentation
                event = "HIGH_ACCELERATION"
                impact_detected = True
                impact_frame = frame
            
            # Detect rebound (after impact)
            if impact_detected and frame > impact_frame and event == "":
                if velocity.y < 0 and curr_pos.y < start_pos.y - 1:  # Moving away from barrier
                    event = "REBOUND"
            
            # Write data to CSV
            writer.writerow([
                frame, round(time, 4),
                round(curr_pos.x, 4), round(curr_pos.y, 4), round(curr_pos.z, 4),
                round(velocity.x, 4), round(velocity.y, 4), round(velocity.z, 4), round(speed, 4),
                round(acceleration.x, 4), round(acceleration.y, 4), round(acceleration.z, 4), round(accel_magnitude, 4),
                round(force.x, 4), round(force.y, 4), round(force.z, 4), round(force_magnitude, 4),
                round(momentum.x, 4), round(momentum.y, 4), round(momentum.z, 4), round(momentum_magnitude, 4),
                round(KE, 4), round(PE, 4), round(total_energy, 4),
                event
            ])
            
            # Update previous values for next iteration
            prev_pos = curr_pos.copy()
            prev_vel = velocity.copy()
            prev_accel = acceleration.copy()
            prev_dir_y = curr_dir_y
            
            # Print progress
            if frame % 10 == 0:
                print(f"Processed frame {frame}/{scene.frame_end}")
    
    print(f"Physics data collection complete. Data saved to {log_file}")

# Add function to create a visualization of the collected physics data
def create_physics_visualization():
    """
    Creates a visualization curve of the key physics values
    """
    # Skip if we haven't run the data collection yet
    if not os.path.exists(log_file):
        print("No physics data file found. Run collect_physics_data() first.")
        return
    
    # Read the CSV data
    data = []
    with open(log_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            data.append(row)
    
    if not data:
        print("No data found in the physics log file.")
        return
    
    # Create a visual representation of the data in the 3D view
    # This will create curve objects showing the trajectory and key physics values
    
    # Create a curve for the trajectory
    bpy.ops.curve.primitive_nurbs_path_add(enter_editmode=False, location=(0, 0, 0))
    trajectory = bpy.context.active_object
    trajectory.name = "Sphero_Trajectory"
    
    # Remove existing points and add new ones based on position data
    spline = trajectory.data.splines[0]
    spline.points.clear()
    
    # Add points from our collected data
    num_points = len(data)
    spline.points.add(num_points - 1)  # -1 because one point already exists
    
    for i, row in enumerate(data):
        x = float(row['Position_X'])
        y = float(row['Position_Y'])
        z = float(row['Position_Z'])
        spline.points[i].co = (x, y, z, 1)  # NURBS points are 4D
    
    # Add a material to the trajectory
    mat = bpy.data.materials.new(name="TrajectoryMaterial")
    mat.diffuse_color = (1, 0.5, 0, 1)  # Orange
    trajectory.data.materials.append(mat)
    
    # Thicken the curve for visibility
    trajectory.data.bevel_depth = 0.05
    
    print("Physics visualization created in 3D view")

# Run the data collection after the simulation is complete
def run_simulation_and_collect_data():
    """Main function to run the simulation and collect physics data"""
    # Set up the simulation
    # (your existing setup code)
    
    # Bake the physics simulation
    bpy.ops.ptcache.bake_all(bake=True)
    
    # Now collect the data after the physics is baked
    collect_physics_data()
    
    # Create visualization
    create_physics_visualization()
    
    # Render if needed
    # bpy.ops.render.render(animation=True)

# Add this to the end of your script
if __name__ == "__main__":
    run_simulation_and_collect_data()

# For direct Blender execution, uncomment:
run_simulation_and_collect_data()

# Ensure we free physics cache when done to prevent memory issues
bpy.ops.ptcache.free_bake_all()
