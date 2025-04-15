import os
import csv
import hashlib

def get_version_folder(modules):
    EXPORT_PATH = "outputs"
    os.makedirs(EXPORT_PATH, exist_ok=True)
    hash_input = "".join(modules).encode('utf-8')
    version_hash = hashlib.md5(hash_input).hexdigest()[:8]
    version_base = f"version_{version_hash}"
    version_path = os.path.join(EXPORT_PATH, version_base)

    if not os.path.exists(version_path):
        os.makedirs(version_path)
        readme_path = os.path.join(version_path, "readme.txt")
        with open(readme_path, 'w') as f:
            f.write("Module configuration:\n")
            for module in modules:
                f.write(f"{module}\n")

    count = 1
    while os.path.exists(os.path.join(version_path, f"run_{count:02d}")):
        count += 1
    run_folder = os.path.join(version_path, f"run_{count:02d}")
    os.makedirs(run_folder)
    return run_folder

def save_episode_csv(log, output_path):
    if not log:
        return
    keys = log[0].keys()
    with open(output_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(log)
