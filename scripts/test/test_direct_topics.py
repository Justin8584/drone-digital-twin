#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time

class TopicLister(Node):
    def __init__(self):
        super().__init__('topic_lister')
        
        # Wait a moment for discovery
        time.sleep(2)
        
        # Get all topics
        topic_list = self.get_topic_names_and_types()
        
        print("\n=== Available ROS2 Topics ===")
        px4_topics = []
        other_topics = []
        
        for topic, types in topic_list:
            if '/fmu/' in topic:
                px4_topics.append(f"{topic} [{types[0]}]")
            else:
                other_topics.append(f"{topic} [{types[0]}]")
        
        if px4_topics:
            print(f"\n✓ Found {len(px4_topics)} PX4 topics:")
            for topic in sorted(px4_topics):
                print(f"  {topic}")
        else:
            print("\n✗ No PX4 topics found")
            
        if other_topics:
            print(f"\n  Other topics ({len(other_topics)}):")
            for topic in sorted(other_topics)[:5]:
                print(f"  {topic}")

def main():
    rclpy.init()
    node = TopicLister()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
