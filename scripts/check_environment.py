#!/usr/bin/env python3
import importlib.util
import shutil

def main():
    checks = {'ros2': shutil.which('ros2') is not None, 'colcon': shutil.which('colcon') is not None, 'gazebo': shutil.which('gazebo') is not None, 'python3': shutil.which('python3') is not None, 'rclpy': importlib.util.find_spec('rclpy') is not None}
    for name, ok in checks.items(): print(f'{name}: {"OK" if ok else "MISSING"}')
    print('Gazebo is optional for the CPU fallback path.')
    return 0 if checks['ros2'] and checks['colcon'] and checks['rclpy'] else 1

if __name__ == '__main__': raise SystemExit(main())
