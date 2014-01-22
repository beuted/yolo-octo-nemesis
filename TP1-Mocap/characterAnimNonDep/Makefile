####
# HEADERS
H_QT = $(shell echo `pkg-config --cflags qt-mt`)
ALL_HDRS = 

####
#LIBRARIES
L_QT = `pkg-config --libs qt-mt`
ALL_LIBS = -L/usr/lib64 -lQGLViewer -lQtXml -lQtOpenGL -lQtGui -lQtCore -lGLU -lGL


####
# COMPILATION
CC      = g++
CFLAGS  = -W -Wall -O3 -ansi -pedantic $(ALL_HDRS) -I./include/ -I./include/QGLViewer/ -I./include/Qt/include/QtCore/ -I./include/Qt/include/QtXml/ -I./include/Qt/include/QtOpenGL/ -I./include/Qt/include/QtGui/
LDFLAGS = $(ALL_LIBS)

EXEC = animation
EXT  = cpp
SRC  = $(wildcard *.$(EXT))
OBJ  = $(SRC:.$(EXT)=.o)


####
# TARGETS
all: $(EXEC)

$(EXEC): $(OBJ)
	$(CC) -o $@ $^ $(LDFLAGS)

%.o: %.cpp
	$(CC) -o $@ -c $< $(CFLAGS)

.PHONY: clean mrproper

clean:
	rm -rf *.o

mrproper: clean
	rm -rf $(EXEC)
