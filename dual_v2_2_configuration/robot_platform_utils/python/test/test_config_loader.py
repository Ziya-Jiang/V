import os
import sys
# Get the directory of your current script and find the repository root
current_dir = os.path.dirname(os.path.abspath(__file__))
py_repo_path = os.path.dirname(current_dir)
repo_parent_path = os.path.dirname(os.path.dirname(py_repo_path))

# Update paths to match your actual cross-platform deployment structure
lib_paths = [
    os.path.join(py_repo_path, "lib", "win_x86", "Release"), # Windows Release build folder
    os.path.join(py_repo_path, "lib", "x86"),                # Linux x86 folder
    os.path.join(py_repo_path, "lib", "arm64"),              # Linux ARM64 folder
]

for path in lib_paths:
    if os.path.exists(path):
        sys.path.append(path)
        
import config_loader

if __name__ == "__main__":
    # Example usage
    yaml_config_path =  fr"{repo_parent_path}\cuarm_configuration\arm_v1\config.yaml"
    yaml_node = config_loader.load_yaml(yaml_config_path)
    yaml_node.print()
    yaml_dict = yaml_node.as_dict()

