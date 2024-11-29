
# Docker Installation and Running ROS Containers (folow these only for the frist time :))

This guide provides step-by-step instructions to install Docker and run a ROS container on different operating systems: **Windows**, **macOS**, and **Ubuntu**.  


---

## For Windows  

### Step 1: Install WSL (Windows Subsystem for Linux)  

1. Open **PowerShell** as Administrator.  
2. Run the following command:  
   ```bash
   wsl --install
   ```
3. Once the installation is complete, reboot your system.  

> **Note**: If you already have WSL installed, you can skip this step and move to the next one.  

### Step 2: Install Docker Desktop  

1. Visit the [Docker installation page for Windows](https://docs.docker.com/desktop/setup/install/windows-install/).  
2. Download the `.exe` file for Docker Desktop.  
3. Run the installer and follow the on-screen instructions to complete the installation.  
4. After installation, reboot your system.  

### Step 3: Running the ROS Container  

1. Open **Docker Desktop** after logging into your system.  
2. In the bottom-right corner, open the **Docker Terminal**.  
   <img width="1372" alt="SCR-20241129-hdwg" src="https://github.com/user-attachments/assets/5f6891df-5274-4f3f-8b52-e6afcd59663d">
3. Run the following command to start the ROS Docker container:  
   ```bash
   docker run -p 6080:80 --security-opt seccomp=unconfined --shm-size=512m saravan29/ros_docker_final:final
   ```
4. Wait for the container to initialize. This may take around 30 minutes for first-time users.  

> **Note**: The first run may take some time to download necessary files and set up the environment.

<img width="784" alt="image" src="https://github.com/user-attachments/assets/e2b2b726-06e6-4ecd-a05e-138732fe72bd">


### Step 4: Access ROS in the Browser  

1. Open your web browser.  
2. In the address bar, type `localhost:6080` and press Enter.  
3. You should now see your ROS environment running inside Docker.  

---

## For macOS  

### Step 1: Install XQuartz  

1. Download and install XQuartz from [here](https://www.xquartz.org/releases/XQuartz-2.8.1.html).  
2. Follow the on-screen instructions to complete the installation.  

### Step 2: Install Docker Desktop  

1. Visit the [Docker installation page for macOS](https://docs.docker.com/desktop/setup/install/mac-install/).  
2. Download the `.dmg` file for Docker Desktop.  
3. Run the `.dmg` file and follow the on-screen instructions.  

### Step 3: Running the ROS Container  

1. Open **Docker Desktop** after logging into your system.  
2. In the bottom-right corner, open the **Docker Terminal**.  
<img width="1372" alt="SCR-20241129-hdwg" src="https://github.com/user-attachments/assets/5f6891df-5274-4f3f-8b52-e6afcd59663d">

3. Run the following command to start the ROS Docker container:  
   ```bash
   docker run -p 6080:80 --security-opt seccomp=unconfined --shm-size=512m saravan29/ros_docker_final:final
   ```
4. Wait for the container to initialize. This may take around 30 minutes for first-time users.  

> **Note**: The first run may take some time to download necessary files and set up the environment.

<img width="784" alt="image" src="https://github.com/user-attachments/assets/e2b2b726-06e6-4ecd-a05e-138732fe72bd">



### Step 4: Access ROS in the Browser  

1. Open your web browser.  
2. In the address bar, type `localhost:6080` and press Enter.  
3. You should now see your ROS environment running inside Docker.  

---

## For Ubuntu  

> **Note**: If you are using an environment other than GNOME, install GNOME Terminal using the following command:  
> ```bash
> sudo apt install gnome-terminal
> ```

### Step 1: Download Docker Desktop  

1. Download the `.deb` file for Ubuntu from the [Docker installation page](https://docs.docker.com/desktop/setup/install/linux/ubuntu/).  

### Step 2: Install Docker Desktop  

1. Open the terminal and enter the following commands:  
   ```bash
   sudo apt-get update
   sudo apt-get install ./docker-desktop-amd64.deb
   ```
2. Reboot your system.  

### Step 3: Running the ROS Container  

1. Open the **Terminal**.  
2. Run the following command to start the ROS Docker container:  
   ```bash
   docker run -p 6080:80 --security-opt seccomp=unconfined --shm-size=512m saravan29/ros_docker_final:final
   ```
3. Wait for the container to initialize. This may take around 30 minutes for first-time users.  

> **Note**: The first run may take some time to download necessary files and set up the environment.

<img width="784" alt="image" src="https://github.com/user-attachments/assets/e2b2b726-06e6-4ecd-a05e-138732fe72bd">


### Step 4: Access ROS in the Browser  

1. Open your web browser.  
2. In the address bar, type `localhost:6080` and press Enter.  
3. You should now see your ROS environment running inside Docker.  

---
