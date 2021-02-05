CC = g++ -std=c++11

CFLAGS = -I/usr/include/
LIBS = -lopencv_calib3d -lopencv_imgproc -lopencv_contrib -lopencv_legacy -lopencv_core -lopencv_ml -lopencv_features2d -lopencv_objdetect -lopencv_flann -lopencv_video -lopencv_highgui

all: main

main: main.cpp
	$(CC) main.cpp -o main `pkg-config --cflags --libs opencv`

clean:
	$(RM) main *.o *.hpp.gch
