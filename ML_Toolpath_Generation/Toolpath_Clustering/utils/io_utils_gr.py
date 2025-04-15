import os
import csv

def save_path_csv(points, path, headers=("x", "y")):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        writer.writerows(points)

def load_points_csv(path):
    with open(path, newline='') as f:
        reader = csv.reader(f)
        next(reader)  # Skip header
        return [tuple(map(float, row)) for row in reader]
