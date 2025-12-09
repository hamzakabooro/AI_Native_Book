#!/usr/bin/env python3
"""
Synthetic Data Generation Example for Isaac Sim
This script demonstrates how to generate synthetic data from Isaac Sim
"""

import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.sensor import Camera
from omni.replicator.core import random_colours
import numpy as np
import cv2
import carb
import os
from PIL import Image
import omni.replicator.core as rep


def setup_synthetic_data_generation():
    """
    Set up Isaac Sim for synthetic data generation
    """
    # Get the world instance
    world = World(stage_units_in_meters=1.0)
    
    # Add a simple humanoid robot to the scene
    # In a real scenario, you would add your URDF or USD file
    add_reference_to_stage(
        usd_path=get_assets_root_path() + "/Isaac/Robots/Franka/franka_alt.usd",
        prim_path="/World/Robot"
    )
    
    # Create a camera for data collection
    camera = Camera(
        prim_path="/World/Camera",
        position=np.array([1.0, 1.0, 1.0]),
        rotation=np.array([0.5, -0.5, -0.5, 0.5])  # Quaternion
    )
    camera.set_resolution((640, 480))
    
    # Add a ground plane
    ground_plane = world.scene.add_default_ground_plane()
    
    # Add a few objects to create a scene
    world.scene.add_ground_plane()
    
    return world, camera


def generate_rgb_data(world, camera, output_dir, num_images=10):
    """
    Generate RGB images from the simulated environment
    """
    os.makedirs(output_dir, exist_ok=True)
    
    for i in range(num_images):
        # Step the world
        world.step(render=True)
        
        # Get the RGB image
        rgb_data = camera.get_rgb()
        
        # Save the image
        image = Image.fromarray(rgb_data)
        image.save(os.path.join(output_dir, f"rgb_{i:04d}.png"))
        
        # Move the camera slightly for the next image
        camera.set_translation(np.array([
            1.0 + 0.2 * np.sin(i * 0.5),
            1.0 + 0.2 * np.cos(i * 0.5),
            1.0 + 0.1 * np.sin(i * 0.3)
        ]))
    
    print(f"Generated {num_images} RGB images in {output_dir}")


def generate_depth_data(world, camera, output_dir, num_images=10):
    """
    Generate depth images from the simulated environment
    """
    os.makedirs(output_dir, exist_ok=True)
    
    # Add depth sensor
    camera.add_ground_truth_to_frame("depth", "/Isaac/Sensors/DepthCamera")
    
    for i in range(num_images):
        # Step the world
        world.step(render=True)
        
        # Get the depth image
        depth_data = camera.get_ground_truth_data("depth")
        
        # Convert to 16-bit format for storage
        depth_image = (depth_data * 1000).astype(np.uint16)  # Scale for 16-bit storage
        
        # Save the depth image
        depth_pil = Image.fromarray(depth_image)
        depth_pil.save(os.path.join(output_dir, f"depth_{i:04d}.png"))
        
        # Move the camera slightly for the next image
        camera.set_translation(np.array([
            1.0 + 0.2 * np.sin(i * 0.5),
            1.0 + 0.2 * np.cos(i * 0.5),
            1.0 + 0.1 * np.sin(i * 0.3)
        ]))
    
    print(f"Generated {num_images} depth images in {output_dir}")


def generate_segmentation_data(world, camera, output_dir, num_images=10):
    """
    Generate semantic segmentation data from the simulated environment
    """
    os.makedirs(output_dir, exist_ok=True)
    
    # Set up semantic segmentation
    camera.add_ground_truth_to_frame("semantic_segmentation", "/Isaac/Sensors/SegmentationCamera")
    
    for i in range(num_images):
        # Step the world
        world.step(render=True)
        
        # Get the segmentation data
        seg_data = camera.get_ground_truth_data("semantic_segmentation")
        
        # Convert to image format
        seg_image = Image.fromarray(seg_data)
        seg_image.save(os.path.join(output_dir, f"seg_{i:04d}.png"))
        
        # Move the camera slightly for the next image
        camera.set_translation(np.array([
            1.0 + 0.2 * np.sin(i * 0.5),
            1.0 + 0.2 * np.cos(i * 0.5),
            1.0 + 0.1 * np.sin(i * 0.3)
        ]))
    
    print(f"Generated {num_images} segmentation images in {output_dir}")


def main():
    """
    Main function to demonstrate synthetic data generation
    """
    print("Setting up Isaac Sim for synthetic data generation...")
    
    # Setup the environment
    world, camera = setup_synthetic_data_generation()
    
    # Play the world
    world.play()
    
    # Output directory for synthetic data
    output_base = os.path.expanduser("~/synthetic_data_output")
    os.makedirs(output_base, exist_ok=True)
    
    # Generate different types of synthetic data
    generate_rgb_data(world, camera, os.path.join(output_base, "rgb"), num_images=20)
    generate_depth_data(world, camera, os.path.join(output_base, "depth"), num_images=20)
    generate_segmentation_data(world, camera, os.path.join(output_base, "segmentation"), num_images=20)
    
    print("Synthetic data generation completed!")
    print(f"Data saved to: {output_base}")
    
    # Stop the world
    world.stop()


if __name__ == "__main__":
    main()