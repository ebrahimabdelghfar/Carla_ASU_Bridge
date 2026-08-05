#!/usr/bin/env python3
import sys
import os
import numpy as np
import matplotlib.pyplot as plt

def main():
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    else:
        print("Usage: python3 visualize_track.py <path_to_csv>")
        sys.exit(1)

    if not os.path.exists(csv_file):
        print(f"File not found: {csv_file}")
        print("Make sure you have recorded a track first.")
        sys.exit(1)

    try:
        # Read data, skipping comments (#)
        data = np.loadtxt(csv_file, delimiter=',', comments='#')
    except Exception as e:
        print(f"Error loading {csv_file}: {e}")
        sys.exit(1)

    if len(data) < 2:
        print("Track needs at least 2 points to visualize.")
        sys.exit(1)

    x = data[:, 0]
    y = data[:, 1]
    w_right = data[:, 2]
    w_left = data[:, 3]

    # Calculate heading using gradient (vectorized)
    dx = np.gradient(x)
    dy = np.gradient(y)
    heading = np.arctan2(dy, dx)

    # Calculate boundary points
    # Left is +90 deg from heading, Right is -90 deg
    left_x = x - w_left * np.sin(heading)
    left_y = y + w_left * np.cos(heading)
    
    right_x = x + w_right * np.sin(heading)
    right_y = y - w_right * np.cos(heading)

    plt.figure(figsize=(10, 10))
    
    # Plot boundaries and center
    plt.plot(x, y, 'k--', linewidth=1, label='Centerline')
    plt.plot(left_x, left_y, 'b-', linewidth=2, label='Left Boundary')
    plt.plot(right_x, right_y, 'r-', linewidth=2, label='Right Boundary')
    
    # Highlight start and end points to verify loop closure
    plt.scatter(x[0], y[0], color='lime', marker='o', s=150, edgecolor='black', label='Start (P0)', zorder=5)
    plt.scatter(x[-1], y[-1], color='magenta', marker='X', s=150, edgecolor='black', label='End', zorder=5)

    plt.title(f'Track Visualization\n{os.path.basename(csv_file)}')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.legend()
    
    # Ensure axes are equally scaled so the track isn't distorted
    plt.axis('equal')
    plt.grid(True, linestyle=':', alpha=0.7)
    
    print("Close the plot window to exit.")
    plt.show()

if __name__ == '__main__':
    main()
