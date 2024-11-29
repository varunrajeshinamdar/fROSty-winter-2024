
# Docker Installation on Windows

Follow these steps to install Docker and run a ROS container on your Windows machine.

## Step 1: Install WSL (Windows Subsystem for Linux)
If you don't have WSL installed already, follow these steps:

1. Open **PowerShell** as Administrator.
2. Run the following command:
   ```
   wsl --install
   ```
3. Once the installation is complete, reboot your system.

> **Note**: If you already have WSL installed, you can skip this step and move to the next one.

## Step 2: Install Docker Desktop

1. Visit the [Docker installation page for Windows](https://docs.docker.com/desktop/setup/install/windows-install/).
2. Download the `.exe` file for Docker Desktop.
3. Run the installer and follow the on-screen instructions to complete the installation.
4. After installation, reboot your system.

## Step 3: Running the ROS Container

1. Once you've logged into your system, open **Docker Desktop**.
2. In the bottom-right corner, open the **Docker Terminal**.
3. Run the following command to start the ROS Docker container:
   ```bash
   docker run -p 6080:80 --security-opt seccomp=unconfined --shm-size=512m saravan29/ros_docker_final:final
   ```
4. Wait for the container to initialize. This may take around 30 minutes for first-time users.

> **Note**: The first time the container starts, it may take some time to download necessary files and set up the environment.

## Step 4: Access ROS in the Browser

1. Open your web browser.
2. In the address bar, type `localhost:6080` and hit Enter.
3. You should now see your ROS environment running inside Docker.

That's it! You have successfully installed Docker and started a ROS container.
