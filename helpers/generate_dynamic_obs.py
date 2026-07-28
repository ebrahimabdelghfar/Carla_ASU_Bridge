import carla
import random
import math
import time

def main():
    # 1. Connect to the CARLA client
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    # --- Configuration Parameters ---
    NUM_PROPS = 30           # Total number of props to spawn
    MAX_DISTANCE = 10.0      # Maximum radius from the origin (0,0,0) in meters
    SPAWN_HEIGHT = 1.0       # Z-axis drop height to prevent ground clipping
    # --------------------------------

    # 2. Get the blueprint library and filter for props
    blueprint_library = world.get_blueprint_library()
    prop_bps = blueprint_library.filter('static.prop.*')

    spawned_actors = []
    print(f"Spawning up to {NUM_PROPS} dynamic props within {MAX_DISTANCE}m of the origin...")

    try:
        for _ in range(NUM_PROPS):
            # 3. Generate random coordinates using polar math
            # This ensures an even distribution in a circle around the origin
            r = random.uniform(0, MAX_DISTANCE)
            theta = random.uniform(0, 2 * math.pi)

            x = r * math.cos(theta)
            y = r * math.sin(theta)
            z = SPAWN_HEIGHT

            # Create the transform with a random rotation so they fall naturally
            spawn_point = carla.Transform(
                carla.Location(x=x, y=y, z=z),
                carla.Rotation(pitch=random.uniform(0, 360), 
                               yaw=random.uniform(0, 360), 
                               roll=random.uniform(0, 360))
            )

            # Choose a random prop blueprint (e.g., boxes, barrels, trash cans)
            bp = random.choice(prop_bps)

            # 4. Try spawning the actor safely
            # Using try_spawn_actor prevents the script from crashing if there is a collision
            actor = world.try_spawn_actor(bp, spawn_point)

            if actor is not None:
                spawned_actors.append(actor)

                # 5. Make it dynamic: Enable physics
                actor.set_simulate_physics(True)

                # Add a random directional force (impulse) so it moves immediately
                impulse = carla.Vector3D(
                    x=random.uniform(-15.0, 15.0),
                    y=random.uniform(-15.0, 15.0),
                    z=random.uniform(0.0, 5.0) # Give it a slight upward kick
                )
                actor.add_impulse(impulse)

        print(f"Successfully spawned {len(spawned_actors)} props. Watch them move in the simulator!")
        print("Press Ctrl+C in this terminal to clean up and exit.")

        # Keep the script alive so the props remain in the simulator
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nInterrupted by user. Cleaning up actors...")
    
    finally:
        # 6. Destroy actors when exiting to keep the simulation environment clean
        for actor in spawned_actors:
            if actor.is_alive:
                actor.destroy()
        print("Cleanup complete.")

if __name__ == '__main__':
    main()