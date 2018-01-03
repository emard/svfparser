all: svfparser

svfparser: svfparser.c main.c svfparser.h
	gcc svfparser.c main.c -o $@

clean:
	rm -f *.o *~ svfparser