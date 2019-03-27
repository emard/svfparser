all: svfparser

TYPE=print
#TYPE=esp32

svfparser: svfparser.cpp main.cpp svfparser.h jtaghw_$(TYPE).h jtaghw_$(TYPE).cpp
	gcc -g -Wall svfparser.cpp jtaghw_$(TYPE).cpp main.cpp -o $@

clean:
	rm -f *.o *~ svfparser
