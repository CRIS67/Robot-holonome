holonome: web.o dspic.o main.o  
	g++ -o holonome web.o dspic.o main.o -Wall -lwiringPi -pthread 

web.o: web.cpp
	g++ -o web.o -c web.cpp -Wall -lwiringPi -pthread 

dspic.o: dspic.cpp 
	g++ -o dspic.o -c dspic.cpp -Wall -lwiringPi -pthread 

main.o: main.cpp web.hpp dspic.hpp
	g++ -o main.o -c main.cpp -Wall -lwiringPi -pthread 