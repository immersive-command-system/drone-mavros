#!/usr/bin/env python
import rospy


def get_topics(namespace):
    return rospy.get_published_topics(namespace)


