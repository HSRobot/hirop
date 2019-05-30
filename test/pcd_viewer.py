#!/usr/bin/env python

import hirop_perecption.hirop_perception as hirop_perception
import ecto, ecto_pcl
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

tester = hirop_perception.RegionFilters("tester", maxZ=1)

viewer = ecto_pcl.CloudViewer("viewer",
                              window_name="PCD Viewer")

saver = ecto_pcl.PCDWriter("Saver")

plasm.connect(reader[:] >> tester[:], tester[:] >> viewer[:], tester[:] >> saver[:])

if __name__=="__main__":
    sched = ecto.Scheduler(plasm)
    sched.execute(niter=1)
    #sleep 10 seconds and exit.
    time.sleep(10)
