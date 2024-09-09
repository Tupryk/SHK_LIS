BASE = ../../git/botop/rai
BASE2 = ../../git/botop

DEPEND = KOMO Core Geo Kin Gui Optim

LIBS := $(shell pkg-config --libs opencv4)
LIBS += -lpthread -I/usr/include/opencv4 

OBJS = main.o src/CameraRecorder.o 
CPPFLAGS += -I/usr/local/include/opencv4
CXXFLAGS += -I/usr/local/include/opencv4

include $(BASE)/_make/generic.mk
