

from roboflow import Roboflow

rf = Roboflow(api_key="MTifXxNCg5Rd2mSMlhCJ")
project = rf.workspace("jb-edsws").project("surface-detecting-bdnua")
version = project.version(4)
dataset = version.download("yolov8")

