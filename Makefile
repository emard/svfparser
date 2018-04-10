all: svfparser

svfparser: svfparser.c main.c svfparser.h
	gcc -g -Wall svfparser.c main.c -o $@

clean:
	rm -f *.o *~ svfparser