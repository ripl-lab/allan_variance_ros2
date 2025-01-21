# How to use a devcontainer for development

1. Install the [Remote - Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension in VSCode
2. Install Docker on local machine (https://docs.docker.com/desktop/setup/install/linux/ubuntu/)
3. Open the allan_variance_ros2 repository in VSCode
4. Reopen in Container: Use the Command Palette (Ctrl+Shift+P or Cmd+Shift+P on macOS) and select "Remote-Containers: Reopen in Container". This will build the container using the Dockerfile and open your project inside the container
5. Run `colcon build` to build the workspace: `cd ~/ros2_ws && colcon build`
5. Run `python3 src/allan_variance_computer.py --input_mcap mcaps/imu_data.mcap --output_dir allan_variance --imu_config_yaml config/realsense_d425i.yaml` to run the allan variance computer
6. Run `python3 src/allan_variance_analysis.py --data allan_variance/allan_variance.csv` to run the allan variance analysis

Once the repository is open in the container, you can develop as normal following the instructions in the [README.md](../README.md) file.
