CC			= gcc
CFLAGS 		= -g
LFLAGS 		= -lbluetooth -lpthread -lm
IFLAGS		= -I. -I/usr/local/bin/sources/
ODIR		= ./obj/
LDIR		= ./lib/
SRC			= $(wildcard $(LDIR)*.c)
LIB_OBJ		= $(wildcard /usr/local/bin/objects/*)
OBJ			= $(notdir $(patsubst %.c,%.o,$(SRC)))
OBJD		= $(addprefix $(ODIR),$(OBJ))
EXEC		?= bt_main7
EXECO		= $(addprefix $(ODIR),$(addsuffix .o,$(EXEC)))

.PHONY: all clean ultraclean

all: $(EXEC)

$(EXEC): % : $(OBJD) $(ODIR)%.o
	$(CC) -o $@ $^ $(LIB_OBJ) $(LFLAGS)

$(OBJD): $(ODIR)%.o : $(LDIR)%.c $(LDIR)%.h
	@ mkdir -p obj
	$(CC) $(IFLAGS) $(CFLAGS) -o $@ -c $<

$(EXECO): $(ODIR)%.o : %.c
	$(CC) $(IFLAGS) $(CFLAGS) -o $@ -c $<

clean:
	rm -f *.o $(ODIR)*.o
	rm -f -r $(ODIR)

ultraclean: clean
	rm -f $(EXEC)
