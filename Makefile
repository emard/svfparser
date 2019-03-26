all: svfparser

TYPE=print
#TYPE=esp32

svfparser: svfparser.cpp main.cpp svfparser.h jtaghw_$(TYPE).h
	gcc -g -Wall svfparser.cpp jtaghw_$(TYPE).cpp main.cpp -o $@

clean:
	rm -f *.o *~ svfparser
