INC_DIR= ../../include
LIB_DIR= ../../lib
BIN_DIR= ../../bin

LDFLAG=-L/usr/X11R6/lib -L/usr/local/lib -L$(LIB_DIR)
LIBS= -lARgsub_lite -lARvideo -lAR -lpthread -lglut -lGLU -lGL -lXi -lX11 -lm
CFLAG= -O -I/usr/X11R6/include -I$(INC_DIR)


all: $(BIN_DIR)/mk_patt


$(BIN_DIR)/mk_patt: mk_patt.c
	cc -o $(BIN_DIR)/mk_patt $(CFLAG) mk_patt.c\
	   $(LDFLAG) $(LIBS)

clean:
	rm -f $(BIN_DIR)/mk_patt

allclean:
	rm -f $(BIN_DIR)/mk_patt
	rm -f Makefile
