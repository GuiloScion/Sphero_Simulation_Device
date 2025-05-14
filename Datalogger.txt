%%writefile /content/sphero_simulation.py

# [PASTE YOUR ORIGINAL SCRIPT HERE]

# Add the physics data collector code at the end
import bpy
import csv
import os
import math
from mathutils import Vector

# Configuration
scene = bpy.context.scene
fps = scene.render.fps
dt = 1.0 / fps # Time step between frames
mass = 0.5 # kg (mass of Sphero)
g = 9.81 # m/s²
log_file = "/content/drive/MyDrive/SpheroRender/sphero_physics_log.csv"

# Ensure the output directory exists
os.makedirs(os.path.dirname(log_file), exist_ok=True)

# Enhanced data collection class
class PhysicsDataCollector:
    def __init__(self, object_name, mass=0.5):
        self.obj = bpy.data.objects.get(object_name)
        if not self.obj:
            raise ValueError(f"Object '{object_name}' not found!")
            
        self.mass = mass
        self.frame_history = {} # Store data for each frame
        self.start_frame = None
        self.impact_frame = None
        self.impact_detected = False
        self.max_displacement = 0
        self.start_pos = None
        self.prev_dir = None
        
        # Initialize CSV file
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
                'Impact_Displacement', 'Rebound_Velocity',
                'Event'
            ])
    
    def get_velocity(self, frame):
        """Get velocity from Blender physics if available, or calculate from positions"""
        if hasattr(self.obj, 'rigid_body') and self.obj.rigid_body:
            # Try to get velocity directly from rigid body simulation
            try:
                # Need to ensure the simulation is evaluated at this frame
                scene = bpy.context.scene
                scene.frame_set(frame)
                bpy.context.view_layer.update()
                
                # Get velocity from rigid body if available
                if hasattr(self.obj.rigid_body, 'linear_velocity'):
                    return Vector(self.obj.rigid_body.linear_velocity)
            except:
                pass # Fall back to position-based calculation
        
        # Calculate velocity from position differences as fallback
        if frame > scene.frame_start:
            prev_frame = frame - 1
            if prev_frame in self.frame_history:
                curr_pos = self.obj.location.copy()
                prev_pos = self.frame_history[prev_frame]['position']
                return (curr_pos - prev_pos) / dt
        
        # Default velocity if we can't calculate it yet
        return Vector((0, 0, 0))
    
    def get_acceleration(self, frame, velocity):
        """Calculate acceleration from velocity changes"""
        if frame > scene.frame_start + 1:
            prev_frame = frame - 1
            if prev_frame in self.frame_history:
                prev_vel = self.frame_history[prev_frame]['velocity']
                return (velocity - prev_vel) / dt
        
        return Vector((0, 0, 0))
    
    def detect_impact(self, frame, velocity, acceleration):
        """Detect impact events based on velocity and acceleration changes"""
        # Initialize event string
        event = ""
        
        # Get acceleration magnitude
        accel_magnitude = acceleration.length
        
        # Get current direction (simplified to just y component for our example)
        current_dir = 1 if velocity.y > 0 else -1
        
        # Detect impact through several methods
        if frame > scene.frame_start + 1:
            prev_frame = frame - 1
            if prev_frame in self.frame_history:
                prev_vel = self.frame_history[prev_frame]['velocity']
                
                # Method 1: High acceleration spike
                if accel_magnitude > 30: # Threshold determined experimentally
                    if not self.impact_detected:
                        self.impact_detected = True
                        self.impact_frame = frame
                        event = "IMPACT-ACCELERATION"
                
                # Method 2: Direction reversal
                if self.prev_dir is not None and current_dir != self.prev_dir:
                    if not self.impact_detected or frame > self.impact_frame + 10: # Allow new impacts after some frames
                        self.impact_detected = True
                        self.impact_frame = frame
                        event = "IMPACT-DIRECTION"
                
                # Method 3: Sudden velocity drop
                prev_speed = prev_vel.length
                curr_speed = velocity.length
                if prev_speed > 2 and curr_speed < prev_speed * 0.5: # Speed dropped by more than 50%
                    if not self.impact_detected or frame > self.impact_frame + 10:
                        self.impact_detected = True
                        self.impact_frame = frame
                        event = "IMPACT-VELOCITY"
        
        # Update direction for next frame
        self.prev_dir = current_dir
        
        return event
    
    def calculate_displacement(self, position):
        """Calculate displacement from start position"""
        if self.start_pos is None:
            self.start_pos = position.copy()
            return 0
        
        displacement_vector = position - self.start_pos
        return displacement_vector.length
    
    def calculate_energies(self, velocity, position):
        """Calculate kinetic and potential energies"""
        # Kinetic energy: 1/2 * m * v²
        speed = velocity.length
        KE = 0.5 * self.mass * speed ** 2
        
        # Potential energy: m * g * h (relative to starting height)
        height = position.z
        if self.start_pos:
            height -= self.start_pos.z
        PE = self.mass * g * height
        
        return KE, PE
    
    def process_frame(self, frame):
        """Process and log physics data for a single frame"""
        scene = bpy.context.scene
        scene.frame_set(frame)
        bpy.context.view_layer.update()
        
        if self.start_frame is None:
            self.start_frame = frame
        
        # Calculate time in seconds
        time = (frame - scene.frame_start) * dt
        
        # Get current position
        position = self.obj.location.copy()
        
        # Calculate velocity
        velocity = self.get_velocity(frame)
        speed = velocity.length
        
        # Calculate acceleration
        acceleration = self.get_acceleration(frame, velocity)
        accel_magnitude = acceleration.length
        
        # Calculate force (F = ma)
        force = acceleration * self.mass
        force_magnitude = force.length
        
        # Calculate momentum (p = mv)
        momentum = velocity * self.mass
        momentum_magnitude = momentum.length
        
        # Calculate displacement from start
        displacement = self.calculate_displacement(position)
        
        # Calculate energy
        KE, PE = self.calculate_energies(velocity, position)
        total_energy = KE + PE
        
        # Track maximum displacement for rebound analysis
        if displacement > self.max_displacement:
            self.max_displacement = displacement
        
        # Detect impact events
        event = self.detect_impact(frame, velocity, acceleration)
        
        # Calculate rebound velocity (only meaningful after impact)
        rebound_velocity = 0
        if self.impact_detected and frame > self.impact_frame:
            # Simple definition: speed after impact
            rebound_velocity = speed
        
        # Calculate impact displacement (how far object has compressed)
        impact_displacement = 0
        if self.impact_detected:
            impact_pos = self.frame_history.get(self.impact_frame, {}).get('position')
            if impact_pos:
                impact_displacement = (impact_pos - position).length
        
        # Store frame data in history
        self.frame_history[frame] = {
            'position': position.copy(),
            'velocity': velocity.copy(),
            'acceleration': acceleration.copy(),
            'event': event
        }
        
        # Log data to CSV
        with open(log_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                frame, round(time, 4),
                round(position.x, 4), round(position.y, 4), round(position.z, 4),
                round(velocity.x, 4), round(velocity.y, 4), round(velocity.z, 4), round(speed, 4),
                round(acceleration.x, 4), round(acceleration.y, 4), round(acceleration.z, 4), round(accel_magnitude, 4),
                round(force.x, 4), round(force.y, 4), round(force.z, 4), round(force_magnitude, 4),
                round(momentum.x, 4), round(momentum.y, 4), round(momentum.z, 4), round(momentum_magnitude, 4),
                round(KE, 4), round(PE, 4), round(total_energy, 4),
                round(impact_displacement, 4), round(rebound_velocity, 4),
                event
            ])

# Main execution function
def main():
    # Create physics data collector for Sphero
    try:
        sphero_collector = PhysicsDataCollector('Sphero', mass=0.5)
        
        # Process each frame
        for frame in range(scene.frame_start, scene.frame_end + 1):
            sphero_collector.process_frame(frame)
            
            # Print progress
            if frame % 10 == 0:
                print(f"Processed frame {frame}/{scene.frame_end}")
                
        print(f"Physics data logged to {log_file}")
        
    except Exception as e:
        print(f"Error: {e}")

# Make sure to bake physics and run the data collection
bpy.ops.ptcache.bake_all(bake=True)
main()