all: svfparser

svfparser: svfparser.c main.c svfparser.h
	gcc -g svfparser.c main.c -o $@

clean:
	rm -f *.o *~ svfparser