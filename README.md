
# Scrap Detection

## Overview

The Scrap Detection project integrates scanCONTROL 3000-50 sensors to perform 3D profiling of workpieces. The system captures data from two laser sensors positioned at different angles, combines the data to create a comprehensive 3D view, and plots the results. 

## Features

- **Dual Sensor Integration**: Utilizes two scanCONTROL 3000-50 sensors for capturing top and bottom profiles of the workpiece.
- **3D Plotting**: Combines the data from both sensors to generate a detailed 3D plot of the workpiece.
- **PyQt5 GUI**: Provides a user-friendly graphical interface for controlling the measurement process.
- **UDP Communication**: Communicates with B&R Automation Studio via UDP for rotation control.
- **Automated Data Capture**: Automates the process of data capture and plotting.

## Requirements

- Python 3.10 or higher
- PyQt5
- Matplotlib
- NumPy
- pymodbus

## Installation

1. **Clone the Repository**:
    ```bash
    git clone https://github.com/jipsonputhayathu/Scrap-Detection.git
    cd Scrap-Detection
    ```

2. **Install Dependencies**:
    ```bash
    pip install -r requirements.txt
    ```

3. **Configure UDP Communication**:
    Ensure the UDP communication setup in B&R Automation Studio matches the IP and port settings used in this project.

## Usage

1. **Run the Application**:
    ```bash
    python automation_studio.py
    ```

2. **GUI Controls**:
    - **Exposure Time**: Set the exposure time for the sensors.
    - **Profile Frequency**: Set the profile capture frequency.
    - **Start Measurement**: Begin the measurement process and capture data.
    - **Stop Measurement**: Stop the measurement process after capturing the necessary data.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please fork this repository and submit a pull request with your changes.

## Acknowledgements

Special thanks to all the contributors and the community for their support.
