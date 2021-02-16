This is the original source of this code: http://wiki.ros.org/ar_recog. It claims to be under GPLv3, but the original source control is no longer available. We are considering switching to a more modern fiduccial system.

- run getAndBuildARToolkit.py  -- this will download and build. it might crash while building gstreamer support, however that is not important, and not needed on Aqua.
- run rosmake to build the node.


See http://code.google.com/p/brown-ros-pkg/wiki/ar_recog for documentation.
