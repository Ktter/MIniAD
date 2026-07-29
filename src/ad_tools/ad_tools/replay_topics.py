import argparse
def main():
    p=argparse.ArgumentParser(description='Replay a rosbag2 directory through ros2 bag.'); p.add_argument('bag'); p.parse_args(); print('Use: ros2 bag play <bag> --clock')
if __name__=='__main__': main()
