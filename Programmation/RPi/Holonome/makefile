holonome: web.o dspic.o mapGeneration.o dStarLite.o trajectoryHandle.o main.o  
	g++ -o holonome web.o dspic.o mapGeneration.o dStarLite.o trajectoryHandle.o main.o -Wall -lwiringPi -pthread 

web.o: web.cpp
	g++ -o web.o -c web.cpp -Wall -lwiringPi -pthread 

dspic.o: dspic.cpp 
	g++ -o dspic.o -c dspic.cpp -Wall -lwiringPi -pthread 

mapGeneration.o: mapGeneration.cpp 
	g++ -o mapGeneration.o -c mapGeneration.cpp -Wall -lwiringPi -pthread 

dStarLite.o: dStarLite.cpp 
	g++ -o dStarLite.o -c dStarLite.cpp -Wall -lwiringPi -pthread

trajectoryHandle.o: trajectoryHandle.cpp
	g++ -o trajectoryHandle.o -c trajectoryHandle.cpp -Wall -lwiringPi -pthread

main.o: main.cpp web.hpp dspic.hpp mapGeneration.hpp dStarLite.hpp trajectoryHandle.hpp
	g++ -o main.o -c main.cpp -Wall -lwiringPi -pthread 

