
# Docker Installation and Running ROS Containers (folow these only for the frist time :))

This guide provides step-by-step instructions to install Docker and run a ROS container on different operating systems: **Windows**, **macOS**, and **Ubuntu**.  

Final Usage (once the whole setup is done) for Docker: 
![docker](https://github.com/user-attachments/assets/e668dad6-d5c2-4295-a3c5-58a4485238d7)

---
## Index
- [Windows Installation](#for-windows)
- [macOS Installation](#for-macos)
- [Ubuntu Installation](#for-ubuntu)
- [Native Installation , No docker(not recommened)](#ROS-installtion-in-Native-UBUNTU---No-Docker)
  
---


## For Windows  

### Step 1: Install WSL (Windows Subsystem for Linux)  

1. Open **PowerShell** as Administrator.  
2. Run the following command:  
   ```bash
   wsl --install
   ```
3. Once the installation is complete, reboot your system.  

> **Note**: If you already have WSL installed, you can skip this step and move to the next one.You can check if WSL is installed in your system by running `wsl --list --verbose` in the powershell. It will show the list of distros installed in WSL , if you have it already or else it will tell command not found. 

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
   docker run -p 6080:80 --security-opt seccomp=unconfined --shm-size=512m saravan29/frosty_ros_docker:comb1
   ```
4. Wait for the container to initialize. This may take around 30 minutes for first-time users.  

> **Note**: The first run may take some time to download necessary files and set up the environment.

<img width="1372" alt="image" src="https://github.com/user-attachments/assets/0cac3b54-d77f-4c06-b1e7-a021ba9349b6">


### Step 4: Access ROS in the Browser  

1. Open your web browser.  
2. In the address bar, type `localhost:6080` and press Enter.  
3. You should now see your ROS environment running inside Docker.
4. To stop it , run `contol + C` in the terminal and to run the container again , just toggle the play button in the docker desktop , no need to folow the command line code again :))
   <img width="1372" alt="image" src="https://github.com/user-attachments/assets/e68e95d1-2dc6-4fc3-807c-c8f91ab67086">


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
      docker run -p 6080:80 --security-opt seccomp=unconfined --shm-size=512m saravan29/frosty_ros_docker:comb1
   ```
4. Wait for the container to initialize. This may take around 30 minutes for first-time users.  

> **Note**: The first run may take some time to download necessary files and set up the environment.

<img width="1372" alt="image" src="https://github.com/user-attachments/assets/0cac3b54-d77f-4c06-b1e7-a021ba9349b6">



### Step 4: Access ROS in the Browser  

1. Open your web browser.  
2. In the address bar, type `localhost:6080` and press Enter.  
3. You should now see your ROS environment running inside Docker.
4. To stop it , run `contol + C` in the terminal and to run the container again , just toggle the play button in the docker desktop , no need to folow the command line code again :))
   <img width="1372" alt="image" src="https://github.com/user-attachments/assets/e68e95d1-2dc6-4fc3-807c-c8f91ab67086">


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
      docker run -p 6080:80 --security-opt seccomp=unconfined --shm-size=512m saravan29/frosty_ros_docker:comb1
   ```
3. Wait for the container to initialize. This may take around 30 minutes for first-time users.  

> **Note**: The first run may take some time to download necessary files and set up the environment.

<img width="1372" alt="image" src="https://github.com/user-attachments/assets/0cac3b54-d77f-4c06-b1e7-a021ba9349b6">


### Step 4: Access ROS in the Browser  

1. Open your web browser.  
2. In the address bar, type `localhost:6080` and press Enter.  
3. You should now see your ROS environment running inside Docker.
4. To stop it , run `contol + C` in the terminal and to run the container again , just toggle the play button in the docker desktop , no need to folow the command line code again :))
   <img width="1372" alt="image" src="https://github.com/user-attachments/assets/e68e95d1-2dc6-4fc3-807c-c8f91ab67086">


---

### ROS installtion in Native UBUNTU - No Docker (Proceed at your own risk!):

The above method of using using ROS is more then enough , EASY and well organizesd for the bootcamp , but if you want to delve more into robotics in future , its advisable to use ROS natively in ubuntu ,you can follow the below steps to install it :

**WARNING** - The following installation procedures can make you do stuff like this ...

<img src="W0_Images/Sherlock_beating.gif" height=300> <img src="W0_Images/Sherlock_screaming.gif" height=300>

or even worse.



* __Ubuntu Installation__ :(( If you are already using ubuntu , then skip to the ROS installation part)

	**Dual-boot**: Follow this [Tutorial](https://www.tomshardware.com/how-to/dual-boot-linux-and-windows-11) or this [Video Tutorial](https://youtu.be/QKn5U2esuRk?si=RP5TieFTjEVU240-) to dual-boot Ubuntu with Windows. For MacOS, follow the procedure in this [video tutorial](https://youtu.be/jbUulXVZIBI?si=XTMyoI4yP6OC0Jc5) </li>
<span style="color:red">[WARNING], Do at your own risk! We will be not responsible if you lose your data. __Follow instructions carefully and make backups before you start!__</span> <br />
> **Note**:  For absolute beginners, we recommend going for any one of the this  installation , we suggest you to go with the docker part mentioned above. Dual-booting can be a little daunting. And you can always opt for dual-booting once you're comfortable with linux.


* __Get familiar with Linux__:
Here are a few additional resources that you can refer to in order to get familiar with Linux:
	* [Video-based Tutorial](https://www.youtube.com/watch?v=IVquJh3DXUA "Introduction to Linux and Basic Linux Commands for Beginners")
	* [Text-based Tutorial](https://ryanstutorials.net/linuxtutorial/ "Linux Tutorial")
  * [Document containing useful linux command lines](https://docs.google.com/document/d/1aroDJBIP-mqYovI8sVYYjGrn_1ugpN5NBauLLihvEjM/edit?usp=sharing)
 


* __ROS Installation/setup__:
        - We will be using ROS2 Humble for the bootcamp , because its well documented and has a large coumminunity . So before you install ROS2 humble in your system , make sure you dont have any previous installs of ROS (even the faulty installs ) for that you can run the below commands.
```bash
dpkg -l | grep ros-* 
```
- To check which version of ros is installed. if you get any output other then humble then follow the below commads to remove it
```bash
sudo apt remove ~nros-<your-ros-version like>-*
```
then 
```bash
sudo apt autoremove
```
  
- Ubuntu 22.04: [ROS2 HUMBLE](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
Go to a particular link and put your first step in the world of ROS.

If you don’t want to have to source the setup file every time you open a new shell, then you can add the command to your shell startup script:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```


