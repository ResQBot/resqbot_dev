## Res.Q Bots Setup

Dieses Tutorial ist eine Schritt für Schritt Anleitung, um die komplette Softwaresuite des Res.Q Bots Teams einzurichten und mit dem Coden zu starten.

**Disclaimer:** This REEADME is meant for internal use by Res.Q Bots members. While we do not forbid anyone else from using it, we want to caution you, that this file is not complete and updated infrequently. It is by no means a reliable source to work with, if you are not in direct contact with one of the people who have been working on it. Most of the packages used are open source packages developrd by other people, who know what they are doing way better than us.
We have done our best to link to the original GitHubs for any non standard ROS packages.



## Inhalt

1. [Ubuntu Grundlagen](#0-ubuntu-terminals)  
2. [Betriebssystem](#1-betriebssystem)  
3. [ROS Installation](#2-ros2-humble)  
4. [ROS Packages](#3-ros-packages)  
	- [Package Liste](#package-liste)  
	- [Packages installieren und einrichten](#packages-installieren-und-einrichten)
5. [Arduino Packages](#4-arduino-packages)

## 1. Betriebssystem

### 1.1 Raspberry Pi 5

**RaspberryPi Imager** installieren: 

	sudo apt update
	sudo apt install rpi-imager

**Imager** starten und Image auf SD-Karte spielen: 

	Raspberry Pi Device = Raspberry Pi 5
	Operating System 	= Other General Purpose OS -> Ubuntu -> Ubuntu Server 24.04 LTS
	Storage 			= SD-Karte

Mit ```Next``` weiter, dann ```EDDIT SETTINGS```.

Im Reiter ```GENERAL```:

	Set hostname: 				sinnvollen Namen auswählen
	
	Set username and password
					Username:	resqbots
					Password:	resq
	
	Configure wireless LAN
						SSID:	ResQBots_5G
					Password:	Res.QBots
	
	Set locale settings
					Time zone: 	Europe/Vienna
			Keyboard layout: 	de 

Im Reiter ```SERVICES```:

	Enable SSH
	Use password authentication

```Save``` und mit ```YES``` bestätigen. Den Warnhinweis mit ```YES``` bestätigen und der Prozess startet. Dies kann einige Zeit dauern. Ist der Prozess beendet, die Karte aus dem Rechner entnehmen und in den Raspi stecken. Dann den Raspi starten.
	
&nbsp;

### 1.2 PC

**Ubuntu 24.04 LTS** Image von Ubuntu Website downloaden.

USB-Startmedium erstellen:
**Rufus**, **Balena-Etcher**, oder ein Ähnliches Tool downloaden und starten. Den Anleitungen folgen. Hier kann nicht viel falsch gemacht werden, solange **Ubuntu 24.04 Desktop** als image gewählt wird.

#### Dual-Boot einrichten
Dieser Abschnitt folg noch

&nbsp;

## 2. Ubuntu Terminals
Ein sehr vereinfachtes und unvollständiges Intro in die **Linux Shell**, das den Umgang mit **Befehlsterminals** lehren und erklären soll.   
In Ubuntu lässt sich fast alles mit Terminalbefehlen erledigen. Gerade fürs Programmieren und den Umgang mit ROS ist es von enormer Bedeutung das Terminal nutzen zu können. 

&nbsp;

### 2.1 Terminal UI
Die Tastenkombination ```Strg```+```Alt```+```T``` öffnet ein neues Terminal. Links in der aktuellen Zeile wird angezeigt, welcher **User** den Befehl ausführt, auf welchem **Host** der Befehl ausgeführt wird und in welchem **Verzeichnis** man sich gerade befindet, z.B.:

	resqbots@Q-T-Pi:~$ echo "hello world"

**resqbots** ist der **User**   

**Q-T-Pi** ist der **Host** ( = das Gerät auf dem der User angemeldet ist)

**~** ist default Verzeichnis beim einloggen

**$** ist ein Trennsymbol, danach fängt der Befehl an

```echo "hello world"``` ist der Befehl, der ausgeführt werden soll

&nbsp;

### 2.2 Wichtige Befehle

### Strg + C
Die Tastenkombination ```Strg``` + ```C``` bricht den aktuell laufenden Prozess ab. Dies ist besonders wichtig, um ROS-Packages zu stoppen.

### cd

```cd <Verzeichnisname>``` wechselt vom aktuellen Verzeichnis in das gewälte Unterverzeichnis. Mit ```/``` können mehrere Ebenen an Unterverzeichnissen aneinander gereiht werden. z.B.:

	cd ros_ws/src/resqbots_drive_interface/

Der Verzeichnisname ```..``` wechselt ein Verzeichnis nach "oben". So kommt man mit

	cd ..

wieder in das Verzeichnis ```ros_ws/src/``` und mit 

	cd ../..
wäre man im Verzeichnis ```ros_ws```

Der Befehl ```cd``` ohne Verzeichnisnamen führt immer in das Standardverzeichnis zurück, egal wo man vorher war.

Das ```~``` Symbol ist ein Platzhalter für den "standard" Arbeitsbereich. Wenn man also beispielsweise im aus dem Verzeichnis ```lidar_ws/src``` in das Verzeichnis ```moveit2_ws/src``` echseln will, gelingt das mit:

	cd ~/moveit2_ws/src

### sudo
```sudo``` steht kurz für **super user do** und entspricht dem "Als Administrator ausführen" bei Windows. Viele Befehle können nur mit dem Vorsatz ```sudo``` ausgeführt werden. Auch geschützte Dateien können nur mit ```sudo``` verändert werden.

### apt install
```apt install <Paketname>``` quasi ausschließlich mit ```sudo``` gemeinsam verwendet installiert das gewünschte Paket.

	sudo apt install hollywood

installiert das hollywood Paket. (Ein netter Scherz, wenn man unwissende beeindrucken will.)

### mkdir
```mkdir <Verzeichnisname>``` erstellt das gewünschte Verzeichnis. 

	mkdir Test

erstellt das Verzeichnis ```Test```

Der zusatz ```-p``` sorgt dafür, dass alle Übergeordneten Verzeichnisse unverändert bleiben. Mit ```-p``` können auch Unterverzeichnisse in einem Befehl mit-genereirt werden.

	mkdir -p ros_ws/src

erstellt das Verzeichnis ```src``` im Verzeichnis ```ros_ws```. Falls das Verzeichnis ```ros_ws``` nicht existiert, wird es ebenfalls erstellt.

### rm
```rm <Dateiname>``` wird eigentlich nur mit ```sudo``` davor verwendet. ```sudo rm <Dateiname>``` löscht die benannte Datei.
Mit dem zusatz ```-r``` können auch ganze Verzeichnisse gelöscht werden.

	sudo rm -r ros_ws

löscht den oben erstellten ```ros_ws``` und **alle Unterverzeichnisse**

### nano
```nano``` ist der Texteditor von Ubuntu. Damit können nahezu alle schreibbaren Dateiformate göffnet und bearbeitet werden. Für schreibgeschützte Dateien musst ```sudo``` vorgesetzt werden.
Mit ```nano <Dateiname>``` wird die benannte Datei geöffnet, oder, falls sie nicht existiert, erstellt. Mit ```sudo nano``` erstellte Dateien sind für "normale" User schreibgeschützt und können nur mit ```sudo``` bearbeitet werden.

	sudo nano README.txt

erstellt z.B. eine schreibgeschützte ```README.txt```.

### Tab
Mit der ```Tab``` Taste können befehle automatisch vervollstöndigt werden.  
Mit einem doppelten ```Tab``` werden alle möglichen Optionen angezeigt.

### ssh
```ssh user@host``` stellt eine remote Verbindung zum gewählten Gerät her und loggt sich als der genannte User ein. Der **Host** kann dabei druch die **IP-Adresse** oder den **Gerätenamen** mit dem zusatz ```.local``` angegeben werden.

	ssh resqbots@q-t-pi.local

loggt sich z.B. als user **resqbots** auf unserem RaspberryPi namen **Q-T-Pi** ein.  
Will man von einem Gerät aus öfter auf das gleiche andere zugreifen, sollte man mit

	ssh-keygen

ein **ssh public key** erstellt werden und dann mit

	ssh-copy-id user@host

auf das Gerät kopiert werden, dass man per ssh ansteuern will.

### ros2 run & ros2 launch
```ros2``` ist ähnlich wie ```sudo``` eine Vorsilbe, die der Shell sagt, dass sie das Programm mit der ROS2 Umgebung ausführen soll.  

```run``` und ```launch``` sind zwei varianten programme in ROS zu starten. ```run``` startet das Programm in seiner einfachsten Form. ```launch``` startet das **launch file** des Packages und kann oft mit einer vielzahl von Optionen versehen werden.

	ros2 run resqbots_drive_interface drive_interface

startet z.B. unser drive_interface. Das zweite ```drive_interface``` ist die Startoption, die wir einfach nicht sehr kreativ benannt haben.

	ros2 launch tele_op operator.launch.py

startet unser ```tele_op``` programm und die ```joy_node```. Die option ```operator.launch.py``` sagt dem Package, welche Nodes gestartet werden sollen.

Die meisten launch Befehlen enden auf ```.launch``` und ```.py``` für **Python** oder ```.cpp``` für **C++**.

### andere ros2 Befehle

```ros2 node list``` -> Liste aktiver ROS Nodes

```ros2 topic list``` -> Liste aktiver ROS Topics

```ros2 topic echo``` -> Zeigt angegebenes Topic an

```ros2 service list``` -> Liste verfügbarer ROS Services

### ./
```./<Dateiname>``` führt die gewählte Datei als **Shell-Skript** aus, falls das möglich ist. In einem Skript können z.B. mehrere Befehle aneinander gereiht sein, oder Befehle mit vielen Optionen ausgeführt werden.

&nbsp;

### Nützliche Programme

```network-manager``` -> Netzwerk-Management

```net-tools``` -> Netzwerkadapter

```iperf3``` -> Netzwerkgeschwindigkeit

```nmap``` -> Geräte im Netzwerk finden

&nbsp;

## 3. ROS2 Installation

Wir verwenden auf unseren Rechnern ```ros2 humble```. Auf dem Raspberry Pi 5 kann leider nur ```ros2 jazzy``` per ```apt``` installiert werden, daher kommt dort die andere Distro zum Einsatz. Bisher haben wir noch keine Kompatibilitätsprobleme entdekt, sie können aber auftreten.  

Dieser Abschnitt folgt der Installationsanleitung der ```ros2 humble``` Dokumentation

### Locale einrichten:
	sudo apt update && sudo apt install locales
	sudo locale-gen en_US en_US.UTF-8
	sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
	export LANG=en_US.UTF-8

### ROS2 Repo einrichten:
	sudo apt install software-properties-common
	sudo add-apt-repository universe

### ROS2 GPG key einrichten:
	sudo apt update && sudo apt install curl -y
	sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg	

### Repo zur source-Liste hinzufügen:
	echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && 
	echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
	
### dev-tools installieren:
	sudo apt update && sudo apt install ros-dev-tools
	
### ROS installieren:
	sudo apt update && sudo apt upgrade -y
	sudo apt install ros-humble-desktop
	
### ROS einrichten:
Alle Packages in Ros müssen **gesourced** werden, damit sie von der Shell gestartet werden können.
Um ein Package schnell in der aktuellen Shell zu sourcen kann der Befehl ```source $package_ws$/install/setup.bash``` verwendet werden. Wobei ```$package_ws$``` durch den Pfad zum Ordner, in dem der ```colcon-build``` Befehl ausgeführt wurde, ersetzt werden muss. (z.B. ~/ros_ws) 
Falls man den ```colcon-build``` Befehl im aktuellen Ordner verwendet hat, reicht ```source install/setup.bash```.

Um ein Package permanent zu sourcen muss der Befehlt in die **~/.bashrc** Datei geschrieben werden. Das ist eine Setup Datei, die der Shell sagt, was sie vor dem Start alles machen muss. (z.B. ROS sourcen, oder die Hintergrundfarbe ändern.) 

	echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

Die **DOMAIN_ID** sagt ROS welche Geräte in deinem Netzwerk zur gleichen Gruppe gehören. Sie ist per Default auf **0**, kann aber verändert werden. Wir haben sie aus Spaß und zur Sicherheit auf **17** gelegt. 
Auch das muss in der Shell eingestellt werden:

	echo "export ROS_DOMAIN_ID=17" >> ~/.bashrc

**rosdep** ist ein Verzeichnis in dem viele Packages über alle aktiven ROS Distros hinterlegt sind. Manchmal hilft es, das **rosdep** zur Verfügung zu haben.

	sudo rosdep init && rosdep update

Zuletzt sollte ein **Workspace** erstellt werden, in dem an eigenen Packages gearbeitet werden kann:

	cd && mkdir -p ros_ws/src

Grundsätzlich werden ROS Packages, die nicht einfach per ```sudo apt install``` installiert werden können in **Workspaces** gespeichert und gebuildet, damit das Dateimanagement vereinfacht wird. Die Workspaces sinnvoll zu benennen ist dabei von Vorteil.

&nbsp;

## 4. ROS Packages

### Package Liste

**Fernsteuerung**  
```joy``` -> liest Controllerdaten aus   
```lotti_teleop``` -> wandelt Controllerdaten in Bewegungsbefehle um  
```ros2_control``` und ```ros2_controller``` -> Motoren ansteuern und Feedback einholen  
```robot_state_publisher``` -> zeigt Position und Lage des Roboters an  
```lotti_control``` -> ros2_control Package für den "Lotti" Roboter  
```lotti_flipper_controller``` -> Controller für Flipper

**Kameras**  
```camera_ros``` -> USB Kamera Interface

**Ton**  
```audio_common``` -> Micro und Lautsprecher

**LiDAR**  
[```Fast_LIO```](https://github.com/hku-mars/FAST_LIO) -> LiDAR Mapping  
```PCL``` -> support für ```Fast_LIO```  
```Eigen``` -> support für ```Fast_LIO```  
[```Livox_SDK2```](https://github.com/Livox-SDK/Livox-SDK)-> LiDAR Interface  
[```livox_ros_driver2```](https://github.com/Livox-SDK/livox_ros_driver2) -> LiDAR Interface  
```nav2_map_server``` -> speichert LiDAR Karten

**Anzeige**  
```rviz2``` -> LiDAR Karte und robot_state  
```rqt_image_view``` -> zeigt Kamerabilder an

&nbsp;

## Packages installieren und einrichten

[Steuerung](#steuerung)  
[Kameras](#webcams)  
[Ton](#audio_common)  
[LiDAR](#lidar-livox-mid-360)

### audio_common
Dieses Setup ermöglicht die Audio-Kommunikation über ROS 2 Humble zwischen einem Laptop und einem Raspberry Pi – in beide Richtungen:  
Laptop → Raspberry Pi (Audioaufnahme am Laptop, Wiedergabe am Pi)  
Raspberry Pi → Laptop (Audioaufnahme am Pi, Wiedergabe am Laptop)

**Installation: Audio-Tools und Dev-Packages**

	sudo apt install -y 
  	libasound2-dev 
  	gstreamer1.0-plugins-base 
 	libgstreamer1.0-dev 
 	liborc-0.4-dev 
  	libdw-dev libelf-dev libunwind-dev
	espeak alsa-utils pavucontrol

**ROS2 Workspace einrichten - auf BEIDEN Geräten!**

	mkdir -p ~/ros2_audio_ws/src
	cd ~/ros2_audio_ws/src
	git clone -b ros2 https://github.com/ros-drivers/audio_common.git

**Abhängigkeiten & Workspace bauen**

	cd ~/ros2_audio_ws
	rosdep install --from-paths src --ignore-src -r -y
	colcon build --symlink-install

**Kommunikation Raspberry Pi → Laptop**

**Raspberry Pi (Sender)**

	export AUDIODEV=plughw:1,0
	source ~/ros2_audio_ws/install/setup.bash
	ros2 run audio_capture audio_capture_node --ros-args -r __node:=capture_pi

**Laptop (Empfänger)**

	source ~/ros2_audio_ws/install/setup.bash
	ros2 run audio_play audio_play_node --ros-args -r __node:=play_laptop

**Verifikation & Tests**

**Verbindung prüfen (z. B. auf dem Pi):**

	ros2 topic info /audio
	
**Erwartete Ausgabe:**

	Type: audio_common_msgs/msg/AudioData
	Publisher count: 1
	Subscription count: 1

**Live-Daten anzeigen:**

	ros2 topic echo /audio

**Testausgabe senden (z. B. am Laptop):**

	espeak "Hallo Raspberry" --stdout | aplay



&nbsp;

## LiDAR: Livox MID 360
Wir arbeiten mit dem **Livox MID 360** LiDAR. Um diesen nutzen zu können müssen auf dem Gerät, das die Daten vom LiDAR auslesen soll die Packages ```Livox_SDK2``` und ```livox_ros_driver2``` installiert sein.
Um die Daten vom LiDAR zu einer Karte zu verarbeiten, verwenden wir ```Fast_LIO```. Das wiederum benötigt ```Eigen3``` und ```PCL``` um zu funktionieren.

Vor der Installation sollte ein **Workspace** für die Livox Packages erstellt werden:

	cd  && mkdir -p livox_ws/src 


### Livox_SDK2
Das GitHub Repo der **SDK** in den **Workspaca** kopieren

	cd ~/livox_ws/src
	git clone https://github.com/Livox-SDK/Livox-SDK2.git
	
Da die **SDK** ursprünglich für frühere ROS Versionen entwickelt wurde müssen vor dem builden noch Änderungen vorgenommen werden. Dazu wird die Zeile ```#include <cstdint>``` in den Bereichen mit den anderen ```#include``` Zeilen in den Dateien ```sdk_core/comm/define.h``` und ```sdk_core/logger_handler/file_manager.h```  eingefügt: 

	cd Livox-SDK2
	nano sdk_core/comm/define.h
	nano sdk_core/logger_handler/file_manager.h
	
Danach ist das Package bereit für den build-Prozess:

	mkdir build && cd build
	cmake ..
	make -j

Nach die Umsetzung des Befehls ```make -j``` dauert extrem lang und beim RaspberryPi hängt sich wahrscheinlich die ssh Verbindung auf. Nicht wundernn, einfach das Terminal schließen und nach ca. 15 Minuten neu verbinden und weiter machen. 

**Fals Verbindung abgebrochen**

ssh aufbauen, dann 
	
	cd livox_ws/src/Livox-SDK2/build

**weiter builden**

	sudo make install

Auch der build-Prozess dauer ewig, nicht wundern. ```Livox_SDK2``` ist nur ein support Package für ```livox_ros_driver2``` und muss nicht gesourced werden.


### livox_ros_driver2 
Das GitHup Repo in den **Workspace** kopieren

	cd ~/livox_ws/src
	git clone https://github.com/Livox-SDK/livox_ros_driver2.git

und dann builden

	cd livox_ros_driver2
	./build.sh humble

Der Parameter ```humble``` funktioniert auch bei humble, das Package ist einfach etwas älter.

Nach dem erfolgreichen Builden das Package sourcen:

	source ~/livox_ws/install/setup.bash
	echo "source ~/livox_ws/install/setup.bash" >> ~/.bashrc
	

### PCL
PCL ist eine Support-Suite, die viele Mathematische Prozesse im Bezug auf Punktewolken übernimmt.
Sie ist einfach zu installieren:

	sudo apt update
	sudo apt install pcl-*
	sudo apt install libpc-dev libpcl-ros-dev

### Eigen3 
Eigen ist eine Sammlung an Header-Files, die den build Prozess anderer Packages unterstützen kann. Sie wird benötigt um ```Fast_LIO``` builden zu können.

Für Eigen sollte ein extra **Workspace** erstellt werden, da es sonst zu ungewollten interaktionen mit anderen Packages kommen könnte. Der restliche Installationsprozess läuft wie immer ab.

	git clone https://gitlab.com/libeigen/eigen.git
	cd eigen
	mkdir build && cd build
	cmake ..
	make -j
	sudo make install

Auch hier ist ein **sourcen** nicht nötig.

### Fast_LIO
Zu guter Letzt kann endlich ```Fast_LIO``` installiert werden.

```Fast_LIO``` ist ein Package, das LiDAR Daten zu einer Karte zusammenfügen kann, ohne dafür **Odometry** Daten zu benötigen, indem es ähnlich dem Menschen "versteht" , dass sich Wände nicht bewegen, sonder der LiDAR selbst. (Nur mit deutlich mehr Mathe dahinter.)

Auch für ```Fast_LIO``` wird ein eigener **Workspace** erstellt, das das Package wieder lang zum builden braucht und nicht versehentlich neu gebuildet werden sollte, wenn man nur ein eigenes Package testen will.

	cd  && mkdir -p fast_lio_ws/src
	cd fast_lio_ws/src
	git clone https://github.com/Ericsii/FAST_LIO_ROS2.git --recursive
	cd ..
	rosdep install --from-paths src --ignore-src -y

Nachdem das GitHub Repo in den **Workspace** kopiert und durch ```rosdep install``` erweitert wurde, muss noch die ```CMakeLists.txt``` Datei überarbeitet werden um mit der neuen ROS Version zu funktionieren.

	nano src/FAST_LIO_ROS2/CMakeLists.txt

Und an allen Stellen das **c++14** und **c++17** durch **c++20** ersetzen.

Danach kann das Package mit ```colcon``` gebaut und im Anschluss gesourced werden.

	colcon build --symlink-install
	source install/setup.bash
	echo "source ~/fast_lio_ws/install/setup.bash" >> ~/.bashrc

Falls das Package auf einem Gerät installiert wurde, das selbst nicht live die erstellte Karte anzeigen soll muss noch die **launch-Datei** angepasst werden.

	nano install/fast_lio/share/fast_lio/launch/mapping.launch.py

Die Zeile ```ld.add_action(rviz_node)``` mit **#** auskommentieren.

### Nav2_map_server
```Nav2``` ist eine Package Suite für die Roboternavigation. Das Subsystem ```nav2_map_server``` verwenden wir, um die in Fast_LIO erstellten Karten zu speichern.

**Installation:**

	sudo apt update
	sudo apt install ros-humble-nav2-map-server

**Verwendung:**

Karte speichern mit 

	ros2 launch nav2_map_server map_saver.launch 

**|unvollständig|**


### LiDAR Einrichten
Um den Lidar verwenden zu können müssen noch ein paar Schritte erledigt werden.

Die ```livox_ros_driver2``` kann nur mit **statischen IP Adressen** arbeiten. Daher muss im Gerät, das die LiDAR Daten auslesen soll, **DHCP** ausgeschaltet werden. Mit

	cd /etc/netplan/
	ls

werden alle Dateien im Verzeichnis ```netplan``` angezeigt. Normalerweise ist es nur eine, die ```50-cloud-init.yaml``` oder so ähnlich heißt. Diese ```.yaml``` Dateien sind schreibgeschützt, müssen also mit ```sudo``` geöffnet werden.

	sudo nano <Dateiname>

Das Terminal sollte nun ca. so aussehen:

	network:
		wifis:
			wlan0:
				dhcp4: true
				

Unter dem Existierenden Abschnitt für ```wifis``` den folgenden Abschnitt einfügen:

	ethernets:
        eth0:
            dhcp4: false
            addresses:
            - 192.168.1.50/24
            optional: true

Nach dem Speichern und Verlassen der Datei müssen die Änderungen übernommen werden:

	sudo netplan apply

Jetzt hat das Gerät eine **statische IP** von 192.168.1.50 im **Ethernet**. Das Gerät kann jetzt nicht mehr über Ethernet angesteuert werden. Wird das benötigt, einfach das ```dhcp4: false``` durch ```dhcp4: true``` ersetzen.



Jetzt muss in der **Konfig-Datei** die **IP Adresse** festgehalten werden.
Der Befehlt

	nano livox_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json 

zeigt die ```MID360_config.json``` Datei. Sie sieht nach der ersten Installation wie folgt aus:

	{
	"lidar_summary_info" : {
		"lidar_type": 8
	},
	"MID360": {
		"lidar_net_info" : {
		"cmd_data_port": 56100,
		"push_msg_port": 56200,
		"point_data_port": 56300,
		"imu_data_port": 56400,
		"log_data_port": 56500
		},
		"host_net_info" : {
		"cmd_data_ip" : "192.168.1.5",
		"cmd_data_port": 56101,
		"push_msg_ip": "192.168.1.5",
		"push_msg_port": 56201,
		"point_data_ip": "192.168.1.5",
		"point_data_port": 56301,
		"imu_data_ip" : "192.168.1.5",
		"imu_data_port": 56401,
		"log_data_ip" : "",
		"log_data_port": 56501
		}
	},
	"lidar_configs" : [
		{
		"ip" : "192.168.1.12",
		"pcl_data_type" : 1,
		"pattern_mode" : 0,
		"extrinsic_parameter" : {
			"roll": 0.0,
			"pitch": 0.0,
			"yaw": 0.0,
			"x": 0,
			"y": 0,
			"z": 0
		}
		}
	]
	}

Im Abschnitt ```"MID360":``` überall die **IP Adresse** ```192.168.1.5``` durch die ```192.168.1.50``` ersetzen. 

Im Abschnitt ```"lidar_configs" :``` kann die **IP Adresse** des LiDARS eingestellt werden. Um diese zu erfahren, den LiDAR mit einem Netzwerk verbinden, einen PC mit dem gleichen Netzwerk verbinden und

	sudo apt update && sudo apt install nmap
	ip a 	#zeigt deine IP Adresse
	nmap <deine IP Adresse> 	# nur die letzte Zahl ersetzen durch 0/24 

In der Liste sollte irgendwo der LiDAR auftauchen. Die **Default IP** unseres LiDARs ist ```192.168.1.196```. 


### Lidar Optionen
In der Datei ```*/src/FAST_LIO_ROS2/config/mid360.yaml``` können Einstellungen für den LiDAR Betrieb vorgenommen werden. ```*``` steht für die übergeordneten Verzeichnisse. In unserem Fall wahrscheinlich ```~/fast_lio_ws```

	nano ~/fast_lio_ws/src/FAST_LIO_ROS2/config/mid360.yaml

Gibt folgendes aus:

	/**:
		ros__parameters:
			feature_extract_enable: false
			point_filter_num: 3
			max_iteration: 3
			filter_size_surf: 0.5
			filter_size_map: 0.5
			cube_side_length: 1000.0
			runtime_pos_log_enable: false
			map_file_path: "./test.pcd"

			common:
				lid_topic:  "/livox/lidar"
				imu_topic:  "/livox/imu"
				time_sync_en: false         # ONLY turn on when external time synchronization is really >
				time_offset_lidar_to_imu: 0.0 # Time offset between lidar and IMU calibrated by other al>
											# This param will take effect no matter what time_sync_en is>

			preprocess:
				lidar_type: 1                # 1 for Livox serials LiDAR, 2 for Velodyne LiDAR, 3 for ou>
				scan_line:  4
				blind: 0.5
				timestamp_unit: 3
				scan_rate: 10

			mapping:
				acc_cov: 0.1
				gyr_cov: 0.1
				b_acc_cov: 0.0001
				b_gyr_cov: 0.0001
				fov_degree:    360.0
				det_range:     100.0
				extrinsic_est_en:  true      # true: enable the online estimation of IMU-LiDAR extrinsic
				extrinsic_T: [ -0.011, -0.02329, 0.04412 ]
				extrinsic_R: [ 1., 0., 0.,
								0., 1., 0.,
								0., 0., 1.]

			publish:
				path_en: true                # true: publish Path
				effect_map_en: false         # true: publish Effects
				map_en: true                 # true: publish Map cloud
				scan_publish_en:  true       # false: close all the point cloud output
				dense_publish_en: false      # false: low down the points number in a global-frame point>
				scan_bodyframe_pub_en: true  # true: output the point cloud scans in IMU-body-frame

			pcd_save:
				pcd_save_en: true
				interval: -1                 # how many LiDAR frames saved in each pcd file; 
											# -1 : all frames will be saved in ONE pcd file, may lead to>



Bei ```map_file_path:``` den gewünschten Pfad + Dateinamen eingeben z.B. ```"~/maps/latest_map.pcd"```

```scan_rate:``` kann auf 10, 30, 50 oder 100 gesetzt werden und steht für die Abtastrate des LiDAR. **Vorsicht:** je höher die ```scan_rate``` desto mehr Rechenleistung wird für die Karte benötigt und desto schneller füllt sich der Arbeitsspeicher.


&nbsp;

## Webcams
Wir verwenden das ```camera_ros``` Package für unsere USB Kameras.

### Installation
Die Installation ist ganz einfach:

	sudo apt update && sudo apt upgrade
	sudo apt install ros-humble-camera-ros

### Einstellungen



&nbsp;

## Steuerung

### Vorbereitung.

**Zu installierende Programme:**  
```robot_state_publisher``` -> published Position und Lage des Roboters basierend auf URDF-File.  
```joint_state_publisher_gui``` -> lässt alle Gelenke manuel bewegen.  
```xacro``` -> vereinfacht Erstellen von URDF-Files

	sudo apt install ros-humble-robot-state-publisher
	sudo apt install ros-humble-joint-state-publisher-gui
	sudo apt install ros-humble-xacro

**URDF**

Das ```.urdf``` File enthält Informationen über alle Bauteile und Gelenke des Roboters. Zum Erstellen eines einfachen ```.urdf``` Files am besten [diesem](https://articulatedrobotics.xyz/tutorials/ready-for-ros/urdf/) Tutorial folgen.

Ein URDF kann auch aus CAD Dateien exportiert werden. Für SolidWorks einfach [hier](https://github.com/ros/solidworks_urdf_exporter) das Plugin installieren und dem im GitHub verlinkten Tutorial folgen.  

Falls das ```.urdf``` File mit ```xacro``` erstellt wurde, muss es erst mit

	xacro file_name.urdf.xacro > file_name.urdf

zu einem normalen ```.urdf``` File kompiliert werden. Danach 2 Terminals öffnen.

	1. ros2 run robot_state_publisher robot_state_publisher file_name.urdf
	2. ros2 run joint_state_publisher_gui joint_state_publisher_gui

```1.``` publisht die Position und Lage des Robots, ```2.``` published das ```/tf``` Topic und ermöglicht, die Gelenke des Roboters durch Schieber zu bewegen.  


### ros2_control und MoveIt2
Für die Steuerung des Roboters verwenden wir ```ros2_control```. Für die inverse Kinematik und Motorkoordinierung des Arms kommt ```moveit2``` zum Einsatz. 

**Installation:**

```ros2_control``` ist ein Standard-Package in ROS2 und somit einfach zu installieren:

	sudo apt install ros-humble-ros2-control ros-humble-ros2-controller

```moveit2``` ist etwas aufwändiger:  
Vorbereitung:

	sudo apt install python3-rosdep		#rosdep sollte bereits installiert sein
	sudo rosdep init					#rosdep sollte bereits initiallisiert sein
	rosdep update						#vor der Verwendung alles auf den neusten Stand bringen
	sudo apt update
	sudo apt dist-upgrade

Colcon ist bereits in der ROS installation enthalten, aber die Funktion ```--mixin-release``` nicht. Deshalb wird sie installiert:

	sudo apt install python3-colcon-common-extensions
	sudo apt install python3-colcon-mixin
	colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
	colcon mixin update default
	sudo apt install python3-vcstool

Als nächstes wird ein Workspace erstellt, damit MoveIt2 nicht versehentlich neu kompiliert wird, wenn wir an anderen Projekten arbeiten:

	mkdir -p ~/moveit2_ws/src

Nun den Tutorial-Ordner aus dem GitHub Repo installieren:
	
	cd ~/moveit2_ws/src
	git clone --branch humble https://github.com/ros-planning/moveit2_tutorials
	vcs import < moveit2_tutorials/moveit2_tutorials.repos
	sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
	cd ~/moveit2_ws

Das Kompilieren von MoveIt2 dauer extrem lang und ist absurd Resourcenaufwendig. Vor dem Start alle anderen Anwendungen und Prozesse schließen und externe Bildschirme abstecken. Der Prozess wird ca. 30 Minuten dauern, wenn alles gut geht. Evtl. hängt sich der Computer zwischendurch auf, in dem Fall einfach neu Starten und den ```colcon``` Befehl wiederholen. Der Fortschritt bis zum Aufhängen bleibt bestehen. Man kann das also einfach so oft wiederholen, bis das Package fertig kompiliert ist.

	colcon build --mixin-release --symlink-install --executor sequential

Wenn der Prozess erfolgreich beendet wurde, das Package sourcen und in der ```.bashrc``` integrieren.

	source install/setup.bash
	echo "source ~/moveit2_ws/install/setup.bash" >> ~/.bashrc


**Modifikationen**
Wir verwenden das MoveIt2 Modul ```moveit_servo``` um unseren Arm mit dem X-Box Controller zu steuern. Um das Modul mit unsererm ```lotti_teleop``` Package kombinieren zu können, müssen ein paar Modifikationen vorgenommen werden.  

&nbsp;

## 5. Arduino Packages
