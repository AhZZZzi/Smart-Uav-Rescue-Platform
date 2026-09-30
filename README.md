# Smart UAV Rescue Platform

## Introduction

Smart UAV Rescue Platform is a pre-thesis project that explores the application of Unmanned Aerial Vehicles (UAVs) in emergency rescue scenarios.

The project aims to develop a communication platform that connects patients, rescue staff, and UAV systems through a centralized relay server. In addition to the main rescue management platform, the repository also contains several experimental modules related to UAV communication, computer vision, and precision landing.

The work serves as a foundation for future thesis research on UAV-assisted emergency response systems.

## Project Objectives

The main objectives of this project are:

* Develop a centralized communication platform for rescue operations.
* Allow patients to submit emergency rescue requests through a web interface.
* Provide rescue staff with tools for monitoring and managing incidents.
* Investigate methods for integrating UAVs into emergency response workflows.
* Explore computer vision techniques that can support autonomous drone operations.
* Evaluate the feasibility of relay-server-based communication between system components.

## System Overview

The system consists of three primary entities:

### Patients

Patients interact with the system through a web application where they can submit rescue requests and provide information related to emergencies.

### Rescue Staff

Rescue personnel access a management dashboard to monitor incoming requests, coordinate operations, and manage system activities.

### UAV Components

UAV-related modules are responsible for communication, telemetry exchange, and experimental functionalities such as vision-based precision landing.

All communication is routed through a centralized relay server.

## Main Components

### Relay Server

Location:

relay_server/

Responsibilities:

* Handle client connections
* Process rescue requests
* Manage communication between system participants
* Store and retrieve system data
* Support real-time message exchange

The relay server is implemented using FastAPI and WebSocket technologies.

### Web Application

Location:

web_app/

The web application provides separate interfaces for patients and rescue staff.

Patient Interface:

* Submit rescue requests
* Provide emergency information

Staff Interface:

* User authentication
* Request monitoring
* Incident management
* Administrative functions

### UAV Research Modules

Location:

standalone_tests/

This directory contains experimental modules developed during the research process, including:

* Precision landing prototype
* Camera calibration tools
* ArUco marker generation
* Pose estimation experiments
* Computer vision testing

These modules are primarily intended for experimentation and evaluation purposes.

## Technologies Used

Backend Technologies:

* Python
* FastAPI
* WebSocket
* SQLite
* Uvicorn

Frontend Technologies:

* HTML
* CSS
* JavaScript

Computer Vision Technologies:

* OpenCV
* ArUco Marker Detection
* Camera Calibration
* Pose Estimation

UAV Technologies:

* ROS2
* PX4

## Project Structure

```text
Smart-Uav-Rescue-Platform/
│
├── docs/
│   └── Project reports and supporting documents
│
├── relay_server/
│   ├── relay_server.py
│   ├── requirements.txt
│   └── database files
│
├── web_app/
│   ├── patient/
│   │   └── Patient web interface
│   │
│   └── staff/
│       ├── login.html
│       ├── dashboard.html
│       └── admin.html
│
├── standalone_tests/
│   └── precision_landing_prototype/
│       ├── Camera calibration tools
│       ├── Marker generation tools
│       ├── Pose estimation modules
│       └── Precision landing experiments
│
├── ros2_ws/
│   └── ROS2 workspace and UAV-related packages
│
├── px4_arm_setup.sh
├── px4_params.sh
├── arm_px4.sh
│
└── README.md
```

## Installation

Clone the repository:

```bash
git clone https://github.com/AhZZZzi/Smart-Uav-Rescue-Platform.git
```

Move to the project directory:

```bash
cd Smart-Uav-Rescue-Platform
```

Install required dependencies:

```bash
pip install -r relay_server/requirements.txt
```

## Running the Relay Server

Start the relay server using:

```bash
python relay_server/relay_server.py
```

or

```bash
uvicorn relay_server:app --host 0.0.0.0 --port 8000
```

After the server starts successfully, the web applications and connected modules can communicate through the relay server.

## Current Status

This repository represents the implementation and experiments conducted during the pre-thesis stage of the project.

Several modules are still under development and may be modified as the research progresses. The current implementation should be considered a research prototype rather than a production-ready system.

## Future Work

Future development may include:

* Integration with physical UAV platforms
* Autonomous mission planning
* GPS-based navigation support
* Real-time location tracking
* Mobile application development
* Cloud deployment
* Improved system scalability and reliability

## Author

Tran Thi Ngoc Huyen

Pre-Thesis Project

## Acknowledgements

This project was developed as part of a pre-thesis study on UAV-assisted emergency rescue systems and serves as a foundation for future thesis research.
