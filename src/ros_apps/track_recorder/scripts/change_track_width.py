#!/usr/bin/env python3
import sys
import os

def main():
    if len(sys.argv) < 4:
        print("Usage: python3 change_track_width.py <csv_file> <new_w_right> <new_w_left> [output_file]")
        print("Example: python3 change_track_width.py track.csv 5.0 5.0")
        sys.exit(1)
    
    input_file = sys.argv[1]
    
    try:
        new_w_right = float(sys.argv[2])
        new_w_left = float(sys.argv[3])
    except ValueError:
        print("Error: new track widths must be numbers.")
        sys.exit(1)
    
    if len(sys.argv) >= 5:
        output_file = sys.argv[4]
    else:
        # Default to overwriting the input file
        output_file = input_file
        
    if not os.path.exists(input_file):
        print(f"Error: File not found: {input_file}")
        sys.exit(1)
        
    try:
        with open(input_file, 'r') as f:
            lines = f.readlines()
            
        with open(output_file, 'w') as f:
            for line in lines:
                clean_line = line.strip()
                # Keep comments and empty lines intact
                if not clean_line or clean_line.startswith('#'):
                    f.write(clean_line + '\n')
                    continue
                
                parts = clean_line.split(',')
                if len(parts) >= 4:
                    # Keep X and Y exactly as they were
                    parts[2] = str(new_w_right)
                    parts[3] = str(new_w_left)
                    f.write(','.join(parts) + '\n')
                else:
                    f.write(clean_line + '\n')
                    
        print(f"✅ Successfully updated track widths to Right: {new_w_right}m, Left: {new_w_left}m.")
        print(f"📁 Saved to: {output_file}")
        
    except Exception as e:
        print(f"Failed to process file: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
