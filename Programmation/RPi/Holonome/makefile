test: web.o dspic.o mapGeneration.o dStarLite.o Sockets.o main.o  
	g++ -o test web.o dspic.o mapGeneration.o dStarLite.o Sockets.o main.o -Wall -lwiringPi -pthread 

web.o: web.cpp
	g++ -o web.o -c web.cpp -Wall -lwiringPi -pthread 

dspic.o: dspic.cpp 
	g++ -o dspic.o -c dspic.cpp -Wall -lwiringPi -pthread 

mapGeneration.o: mapGeneration.cpp 
	g++ -o mapGeneration.o -c mapGeneration.cpp -Wall -lwiringPi -pthread 

dStarLite.o: dStarLite.cpp 
	g++ -o dStarLite.o -c dStarLite.cpp -Wall -lwiringPi -pthread 

Sockets.o: Sockets.cpp
	g++ -o Sockets.o -c Sockets.cpp -Wall -lwiringPi -pthread 

main.o: main.cpp web.hpp dspic.hpp mapGeneration.hpp dStarLite.hpp Sockets.h
	g++ -o main.o -c main.cpp -Wall -lwiringPi -pthread 



