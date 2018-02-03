CPP = g++
CPPFLAGS = -g -DUSE_LINUX -Wall -Wno-deprecated -pthread -std=c++0x

LIB_DIR = /home/robot/dragonfly/lib
INC_DIR = -I/home/robot/Proficio_Balistic_Release/Dragonfly_Message_defs -I/home/robot/dragonfly/include
TM_DIR = .

Executive: ${TM_DIR}/Executive.cpp
	${CPP} ${CPPFLAGS} -L${LIB_DIR} ${INC_DIR} -o ${TM_DIR}/Executive ${TM_DIR}/Executive.cpp -lDragonfly

all: Consumer

clean:
	rm -f ${TM_DIR}/Executive

