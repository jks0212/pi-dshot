CC      := gcc
CFLAGS  := -O2 -Wall

LIB_SRCS := dshot_esc.c dshot.c
LIB_HDRS := dshot_esc.h dshot.h

all: dshot_test

dshot_test: dshot_test.c $(LIB_SRCS) $(LIB_HDRS)
	$(CC) $(CFLAGS) -o $@ dshot_test.c $(LIB_SRCS)

example: quad_example

quad_example: examples/quad_example.c $(LIB_SRCS) $(LIB_HDRS)
	$(CC) $(CFLAGS) -I. -o $@ examples/quad_example.c $(LIB_SRCS)

clean:
	rm -f dshot_test quad_example

.PHONY: all example clean
