# ROS2 Bag File Demos

This repository contains demo files for working with ROS2 bag files.

## Introduction

ROS2 bag files are a powerful tool for recording and replaying robot sensor data and messages. This repository provides examples and utilities to help you understand and work with ROS2 bag files effectively.

## Prerequisites

- ROS2 (tested with Foxy, Galactic, and Humble)
- Python 3.6+
- rosbag2 package

## Usage

To run the demo `<record_name>`, first copy the file into the container, and use the following command:
```
ros2 bag play bag_files/<record_name>
```

Replace `<record_name>` with the specific demo you want to run (e.g., `vicon_test_1`, `vicon_test_2`

## Troubleshooting

If you encounter any issues, please check the following:

1. Ensure your ROS2 environment is properly set up
2. Verify that all prerequisites are installed
3. Check the console output for any error messages

If the problem persists, please open an issue on the GitHub repository.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
