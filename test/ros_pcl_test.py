#!/usr/bin/env python

import hirop_perecption.hirop_perception as hirop_perception
import ecto, ecto_pcl, ecto_ros
import sys
import time
import os

plasm = ecto.Plasm()
pcdfile = os.path.join(os.path.dirname(__file__),
                       'test.pcd')

if len(sys.argv) > 1:
    pcdfile = sys.argv[1]

reader = ecto_pcl.PCDReader("Reader",
                            filename=pcdfile)

filters = hirop_perception.RegionFilters("tester", maxZ=1)

fusion = hirop_perception.PclFusion()

viewer = ecto_pcl.CloudViewer("viewer",
                              window_name="PCD Viewer")

saver = ecto_pcl.PCDWriter("Saver")

source = hirop_perception.PointCloudRos('source1', topic_name="/camera/depth/points")

plasm.connect(source["output"] >> fusion["input"], fusion["output"] >> viewer["input"])


if __name__=="__main__":
    ecto_ros.init(sys.argv, "ros_test")
    sched = ecto.Scheduler(plasm)
    sched.execute(niter=1)
    fusion.params.test=True
    #sleep 10 seconds and exit.
    time.sleep(10)
    sched.execute(niter=1)
    time.sleep(10)
