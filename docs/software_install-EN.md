# Installation and configuration of the Linux distribution

_**NOTE: Outside classes you can use the Teams or the e-mail to place questions on this subject, to have a faster reply. In case you have some difficulty, I suggest you contact the professor during the attendance hours, or as stated above, so as not to loose too much time with the configuration/installation.**_

The pratical developlent in the Advanced Robotics class is based on ROS2. Even though, nowadays, ROS2 is alredy suported on Windows, typically this type of systems is implemented in a Linux environment, thus we use the same approach in Advanced Robotics. The Linux distribution used in 2022/2023 in Advanced Robotics, and which is installed in the System Simulation Laboratory PCs, is Kubuntu 24.04.02 (Noble Numbat). Although you are not obliged to use Kubuntu, I strongly suggest that, for compatibility reasons, you use one of the 24.04 Ubuntu versions (either Kubuntu, Ubuntu, Xubuntu, etc., being Xubuntu the one that consumes less resources).  At the end of this document you will find the links for Kubuntu and Xubuntu.

You have three advisable ways of installing the Linux distribution, summarized in the followin Table.
|Installation type | Environment | Advantages | Disadvantages |
|:-----------------:|:--------:|:---------:|:------------:|
|[Windows Subsystem for Linux 2 (WSL2)](#1-windows-subsystem-for-linux-2-wsl) | Windows 10/11 | Easy to use and good integration between Windows and Linux. | Performance loss which is low in general, but higher for 3D applications. Only available for  Windows versions >= 10 2004 (Build 19041).|
|[Virtual Machine](#2-virtual-machine) | Windows (7/8/10/11) | Easy to install and uninstall. | Higher performance loss. More suited for computers with at least 4 cores and 8Gb of memory.|
|[Real Machine](#3-real-machine) | Linux Ubuntu 24.04.2 | Higher performance. | Higher installation complexity and more difficult to share date between Windows and Linux - you cannot run both systems simultaneously.|

Each of these options is detailed in the secions below.

You have three advisable ways of installing the Linux distribution, all detailed below.

# 1. Windows Subsystem for Linux 2 (WSL)

This is the recommended option if you use windows and your PC as low resources and you do not want to install Linux alongside Windows. Microsoft has been developing a Linux-based system which allows running Linux applications on a Windows-based environment. This approach, although being based on a virtual machine, it uses less resources that a tradicional virtual machines, and allows working more closely to Windows. The WSL version that we will use in the  Advanced Robotics class is WSL2.

## 1.1 WSL2 Installation

To install WSL2 you need to have Windows 10 2004 (Build 19041) or later, as you can confirm [here](https://learn.microsoft.com/en-us/windows/wsl/install). Proceed as follows to install WSL2 and Ubuntu:
1. Click the start menu and open the option "Turn Windows Features on or off" (you can search for "Turn Windows" to find that option);
2. In the list of available options, activate the options "Windows Subsystem for Linux" and "Virtual Machine Platform", and click OK. You will be asked to restart your PC to conclude the installation;
3. After restarting the PC, open the Microsoft store, access [Ubuntu 24.04.x](https://www.microsoft.com/store/productId/9PN20MSR04DW) and obtain/install the application (Linux distribution to be used);
4. After concluding the Ubuntu 22.04 installation, open the application through the Windows start menu. At that point, you will be asked to introduce a username a password, to configure Ubuntu (do not forget that username/password combination);
5. After confirming that the installation is concluded, close the Ubuntu window and open the PowerShell as an administrator, by right-clicking the start menu, followed by the option "Windows PowerShell (admin)";
6. In the Powershell window, run the command `wsl -l -v` to confirm the WSL version being used;
7. If the version obtained in the previous step is "1", run the command `wsl --set-version Ubuntu-24.04 2` to switch to version 2. If the version is already 2, or if the switch concluded successfully (check with the command from step 6), you can skip the following steps and proceed with the text shown after these numbered steps. Otherwise, proceed with step 8;
8. If in the end of step 7, WSL2 was not activated, that might be due to the need of an additional update. In that case, download and install the update available [here](https://wslstorestorage.blob.core.windows.net/wslblob/wsl_update_x64.msi);
9. Restart your PC for the change to be applied and confirm in your BIOS that the virtualization option is enabled. If your PC does not allow virtualization, you cannot use WSL2, and thus you should use the [virtual machine](#2-virtual-machine) or the [real machine](#3-real-machine) options instead;
10. Confirm again the WSL version used with Ubuntu. If needed, run step 7 again.
11. To make sure you are running the latest available WSL2 version, you can open the Windows console/powershell with administration privileges and execute the following command: `wsl.exe --update`.

Note that you can access the user "HOME" on Linux using Windows Explorer through the path `\\wsl.localhost\Ubuntu-24.04\home\USER`, where `USER` must be replaced by the username you chose on the Linux installation. 

Proceed now with the installation of additional Winows applications that will be useful in Avanced Robotics, as detailed in the next section.

## 1.2 Additional applications to install on Windows

You should install [Notepad++](https://notepad-plus-plus.org/) in case you want to edit files in Windows which are stored in the Linux system.

You need to install [Microsoft Visual Studio Code](https://code.visualstudio.com/) on Windows and, within Visual Studio Code, install the extension [WSL](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-wsl), published by Microsoft.

Having concluded the Linux installation, you now have to install the Advanced Robotics specific applications. To do so, follow the steps described in [Section 4](#4-installing-and-configuring-additional-software).

# 2. Virtual Machine

If you have a reasonably recent PC (< 5 years), this is the recommended option, since it gives a safer and practical experience. Naturally that it has a downside, which is a lower performance. If you have a PC with a processor equal or better than an i5 and an AMD or NVIDIA GPU, or a recent Intel integrated GPU, the performance will be acceptable, even for projects using 3D simulation.

I suggest using [VMware Workstation Pro](#5-relevant-links), which is free for personal use. Alternatively you can use VirtualBox, however, the last version I tested had lower performance when using 3D applications, which was not acceptable when using Gazebo. Using VMware Workstation Pro with 3D applications might not also work well if the PC is not powerful enough. (Note that, currently, we are not using Gazebo, so this is not necessarly an issue.)

A virtual machine is an application which allows running an operating system in a virtual environment, allowing running other operating systems within a host operating system (Windows, Linux or Mac). In these virtual environments, the Host is the operating system where the virtual machines will run, while the Guest is the operating system running on the virtual machine. WSL2 is also a virtual machine, but it os considered a _lightweight_ one.

Since the virtual machine uses a lot of resources, you are advised to use Xubuntu, but you can also use the other stated versions.

Regarding the virtual machine installation, you have two options:
1. You can download a virtual machine previously prepared by the professor (process detailed in [Section 2.1](#21-installation-supplied-by-the-professor));
2. Install the virtual machine from scratch (process detailed in [Section 2.2](#22-installing-the-virtual-machine-from-scratch)). The fastest and simpler way is the first one, but if you wish to go through the steps of installing the Linux virtual machine, you can follow the second option (installing from scratch). In both cases you will be able to copy & paste content between both the host operating system and the guest operating system, as explained in [Section 4](#4-installing-and-configuring-additional-software).

## 2.1 Installation supplied by the professor

In order to proceed with the installation previously set up by the professor, go through the following steps:

1. Download [Vmware Workstation Pro](#5-relevant-links);
2. Install VMware Workstation Pro from the downloaded file (accept the default options, installing the Windows Hypervisor, if your PC supports it);
3. Download the [virtual machine previously set up by the professor](https://drive.google.com/file/d/1xA70rAbAKqbNPy_Y2VMamxIyPy966k3s/view?usp=sharing);
4. Decompress the file (virtual machine) to the folder where the virtual machines are stored in your PC (in the professor's case, it was in `C:\Users\hugoc\Documentos\Virtual Machines\`), or to any other folder of your choosing. After decompressing the files, it will take about 25Gb of hardrive space;
5. Open VMware Workstation Pro;
6. Click in _File_ --> _Open..._ and open, in the folder where you stored the virtual machine, the file "Xubuntu 24.04 64-bit\Xubuntu 24.04 64-bit.vmx";
7. In the message that pops up questioning if the virtual machine was copied or moved, choose the option "I moved it".

The username is `robotics` and the password is `robotics`.

In this case the virtual machine is completely configured and ready to be used, except for the shared folders. Although you can already use Copy&Paste between the host (Windows) and the guest (Linux), it might be useful to share folders between Windows and Linux. To do so, click _File_ --> _Manage_ --> _Virtual Machine Settings_ and select the option "Options", followed by the option "Shared Folders", activate the Always Enabled option and click Add in order to select the folder to be shared. The shared folders will be immediately available in "/mnt/hgfs".

Before proceeding, open the virtual mchine and, in a terminal inside the virtual machine, run the following commands to make sure your system is up to date:
```bash
sudo apt update
sudo apt dist-upgrade
```

You are now ready to follow the laboratory tutorials, you do not need to do anything else in terms of installation/configuration.

## 2.2 Installing the virtual machine from scratch

To install the system as a virtual machine from scratch, proceed ad follows:

1. Download [Vmware Workstation Pro](#5-relevant-links) (click the "Download for free", followed by the "Go to downloads" option, and tehn by the "Download now" option for your operating system;
2. Download [24.04.2 do Xubuntu 64 bit](#5-relevant-links);
3. Install Vmware Workstation Pro from the downloaded file (accept all the default options);
4. Open Vmware Workstation Pro. If you get a window about VMware Workstation Pro, choose "Skip this version";
5. Click "Create a New virtual Machine";
6. Select the option "Installer disc image file (iso)" and choose the Xubuntu ISO you just downloaded. Click Next;
7. Choose the username and password (you cannot forget it) and click Next;
8. Choose a name for the Virtual Machine and click Next;
9. Change the disk size to 25Gb or more and select the option "Store virtual disk as a single file". Click Next;
10. When configuring the virtual machine "hardware", available by clicking on "Customize Hardware...", you should allow for 3GB of memory or more (ideally 4096 Mb, if you have 8Gb of memory or more) and 1 processor or more (ideally 2, if your computer has 4 or more cores). Click Finish to start the installation, which can take 5 to 20 minutes, depending on your hardware;
11. If you get a window with the title "Removable devices", you can close it.

When the installation finishes, we need to do some configurations:

1. If, after the installation, your keyboard is not correctly configured, you can reconfigure it. To do so, search for "Settings Manager" and click it. Click Keyboard. On the Layout tab, remove the option "Use system defaults", click "Edit" and select the option "Portuguese (no dead keys)", or the appropriate one for your keyboard layout, followed by Close;
2. Still in the "Settings Manager" window, search for "Time and Date", click on the icon that appears, then click "Unlock", type the password you have defined previously,  change the "Time zone" to Europe/Lisbon and click "Close" to finish;
3. You can drag and Copy & Paste files between the guest and host, but It can be useful to share one or more folders between the Host and the Guest. To do so, click in the menu _File_ (if the menu is not visible, place the mouse on the topo central part of the virtual machine window) --> Manage --> Virtual Machine Settings. Select the "Options" tab, followed by the option "Shared Folders". Activate the option "Always Enabled" and click "Add" to add the folder to be shared. The "Host path" corresponds to the Windows folder to be shared, while the "Name" corresponds to the same that the shared folder will have on Linux, in the virtual machine. After you fill in these fields, click "Next" followed by "Finish". The shared folder will be immediately available on Linux at `/mnt/hgfs`;
4. Still in the window from the previous item, select the option "VMWare Tools", activate the option "Synchronize guest time with host".
5. By default, Xubuntu locks the screen after 5 minutes of inactivity. If you wish to change this value or disable the lock screen, access the start menu, click Settings, followed by Screensaver Preferences, andchange there the options as desired.

When the configuration is finished, you need to install additional class specific software, as described in [Section 4](#4-installing-and-configuring-additional-software).

# 3. Real Machine

The installation of the Linux distribution on a real machine, side-by-side with Windows (dual-boot configuration), is the one that gives best performance, making full use of the PC resources. However, it implies additional precautions during the installation, specially for less experienced users. It also means that you need to restart your PC whenever you want to switch operating system. I only recommend this option if you fill comfortable with these operations or have an old PC.

To install the distribution you should have a CD or an USB PEN with the distribution on it (you can use the [balenaEtcher](#5-relevant-links) to create a bootable USB PEN from the iso file). You should guarantee that you have at least 2Gb memory and 25Gb available on the harddrive  for Linux (ideally already as free space, that is, already partitioned). Boot that CD/PEN and follow the steps below (based on Kubuntu 22.04, but should be similar for other Ubuntu versions):

1. In the starting menu choose the installation option;
2. Select "Download updates while installing" and "install this third-party software" after booting;
3. When you specify the username and password, activate the option "Log in automatically". Do not forget this username and password, and use only letters and numbers in the username (do not use accents).;
4. Choose carefully the partition scheme for your PC. Talk with the Professor if you have doubts (a mistake here can easily lead to data losses);
5. After the installation go through the following steps:
    * Open "System Settings" in the start menu, click "Locale" and change the country to Portugal. Logout and login for the changes to take place;
    * Open the start menu and activate the option "Driver Manager" (alternatively you can press Alt+F2 and write "Driver Manager"). If there is additional software available, activate it (you might need to restart your PC for the changes to take place).

Having finished the installation, you now need to install additional specific class software, as detailed in [Section 4](#4-installing-and-configuring-additional-software).

# 4. Installing and configuring additional software

To make it easier to install the base software needed for Advanced Robotics, the professor created a set of scripts. As such, in order to install that software, go through the following steps (note that these steps imply downloads and installation of applications occupying over 2Gb of hardrive space):

1. Open a terminal in Linux, or Ubuntu on Windows if you are using WSL;
2. In the terminal, run the 1st command, if you are not using WSL, or the 2nd one, if you are using WSL:
```bash
wget https://github.com/ipleiria-robotics/adv_robotics/raw/master/scripts/install_AR.sh
```
```bash
wget https://github.com/ipleiria-robotics/adv_robotics/raw/master/scripts/install_AR_WSL.sh 
```
3. In the same terminal, run the command `bash install_AR.sh` (or the command `bash install_AR_WSL.sh` if you are using WSL);
4. Follow the instructions shown in the terminal, by typing the password if asked for (you might be asked for the password several times), and answering Yes (Y) or ENTER when asked. Depending on your internet speed and PC, this step can take a long time to finish.

Having finished the installation, check the terminal for possible errors that you might had. If you have any error, besides the one show below, run again the installaation script (step 3). If the errors are not solved that way, get in touch with the professor.

Next, restart your PC, and you are ready fo the lab tutorials and assignments.

In case you are using a virtual machine and wish to share folders between the host operating system and the guest operating system, but not have done so, you should, in VMWare Workstation Pro, access the virtual machine settings, then the Options tab, choose the Shared Folders option, activate the Always enabled option and add the folders you wish to share. The shared folders will be available in the virtual machine in the "/mnt/hgfs" folder.
In case you are using a virtual machine and wish to share folders between the host operating system and the guest operating system, but not have done so, follow the instructions given in the end of [Section 2.1](#21-installation-supplied-by-the-professor).

# 5. Relevant links

* [Kubuntu 24.04.2](https://cdimage.ubuntu.com/kubuntu/releases/24.04.2/release/kubuntu-24.04.2-desktop-amd64.iso)
* [Xubuntu 24.04.2](https://cdimage.ubuntu.com/xubuntu/releases/24.04.2/release/xubuntu-24.04.2-desktop-amd64.iso)
* [Vmware Workstation Pro](https://myipleiria-my.sharepoint.com/:u:/g/personal/hugo_costelha_ipleiria_pt/ETxpWn5RjYlBoZw5I88oT_8B7YV4BGO7hf8Vo2YRFlcvyQ?e=6HtJB6)
* [balenaEtcher](https://etcher.balena.io/)