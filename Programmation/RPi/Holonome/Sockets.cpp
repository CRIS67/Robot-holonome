#include "Sockets.h"
#include <errno.h>
namespace Sockets
{
	bool Start()
	{
#ifdef _WIN32
		WSAData wsaData;
		return WSAStartup(MAKEWORD(2, 2), &wsaData) == 0;
#else
		return true;
#endif
	}
	void Release()
	{
#ifdef _WIN32
		WSACleanup();
#endif
	}
	int GetError()
	{
#ifdef _WIN32
		return WSAGetLastError();
#else
		return errno;
#endif
	}
	void CloseSocket(SOCKET s)
	{
#ifdef _WIN32
		closesocket(s);
#else
		close(s);
#endif
	}

	/*Gets the IP@ of the client*/
	std::string GetAddress(const sockaddr_in& addr)
    {
        char buff[INET6_ADDRSTRLEN] = { 0 };
        return inet_ntop(addr.sin_family, (void*)&(addr.sin_addr), buff, INET6_ADDRSTRLEN);
    }

    /*Starts the server and returns 0 if there is no problem.
    If there is a problem it returns a negative integer s*/
    int startServer(DsPIC dspic)
    {
	        // Socket initialization
		if (!Sockets::Start())
		{
			std::cout << "Erreur initialisation WinSock : " << Sockets::GetError();
			return -1;
		}

		SOCKET server = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP); // TCP socket creation

		if (server == INVALID_SOCKET)
		{
			std::cout << "Erreur initialisation socket : " << Sockets::GetError();
			return -2;
		}

		const unsigned short port = 9999;
		sockaddr_in addr; // socket structure
		addr.sin_addr.s_addr = INADDR_ANY; // any source is accepted
		addr.sin_port = htons(port); // port number has to bee translated to network data
		addr.sin_family = AF_INET; // socket type : TCP

		int res = bind(server, (sockaddr*)&addr, sizeof(addr)); // local port associated to socket
		if (res != 0)
		{
			std::cout << "Erreur bind : " << Sockets::GetError();
			return -3;
		}

		res = listen(server, SOMAXCONN); // socket can listen for connections
		if (res != 0)
		{
			std::cout << "Erreur listen : " << Sockets::GetError();
			return -4;
		}

		std::cout << "Serveur demarre sur le port " << port << std::endl;

	    // Wait for incoming socket
		for (;;)
		{
			sockaddr_in from = { 0 };
			socklen_t addrlen = sizeof(addr);
			SOCKET newClient = accept(server, (sockaddr*)(&from), &addrlen); // waits for new connection
			if (newClient != INVALID_SOCKET)
			{
				std::string clientAddress = Sockets::GetAddress(from);
				std::cout << "Connexion de " << clientAddress << ":" << addr.sin_port << std::endl;
	            Sockets::dataTreatment(newClient,dspic);
			}
			else
			{
	            return -5;
			}
		}
		Sockets::CloseSocket(server);
		Sockets::Release();
		return 0;
    }


    /* - gets the message size and cheks if the message is OK
       - prints the buffer
    */
    int dataTreatment(SOCKET newClient, DsPIC dspic)
    {
        std::vector<unsigned char> buffer;
        int  stopCmd = 0; 

        while(1){
            unsigned short expectedSize;

            int pending = recv(newClient, reinterpret_cast<char*>(&expectedSize), sizeof(expectedSize), 0); // gets the size of the message
            if ( pending <= 0 || pending != sizeof(unsigned short) )
            {
                std::cout << "Error ! Problem with expected size of the message. Client disconnected";
                return -1;
            }

            expectedSize = ntohs(expectedSize); // converts network data to host data
            buffer.resize(expectedSize);
            int receivedSize = 0;
            do {
                int ret = recv(newClient, reinterpret_cast<char*>(&buffer[receivedSize]), (expectedSize - receivedSize) * sizeof(unsigned char), 0);
                if ( ret <= 0 )
                {
                    std::cout << "Error ! Problem getting the message" << std::endl ;
                    buffer.clear();
                    return -2;
                }
                else
                {
                    receivedSize += ret;
                }
            } while ( receivedSize < expectedSize ); // We read only the the amount of data expected

            // Prints buffer to the console
            double x = buffer.at(0)*100 + buffer.at(1)*10 + buffer.at(2);
            double y = buffer.at(4)*100 + buffer.at(5)*10 + buffer.at(6);
            double t = buffer.at(8)*100 + buffer.at(9)*10 + buffer.at(10);
            stopCmd = buffer.at(11); 

            /*for(int i=0; i<buffer.size(); i++)
            {	
            	if(buffer.at(i) != ';')
            		std::cout << (int) static_cast<uint8_t>(buffer.at(i));
        		else 
        			std::cout << buffer.at(i); 


				
            }*/

            x = x * 1 -320;
            y = y * 1-240;
            if (t <= 180) 
              t = t/180.0; 
            else 
              t = (t-360)/180.0; 
            std::cout << "x = " << x << " & y = " << y << " & t " << t  << std::endl;
            if( x <10 && x > -10) 
              x = 0; 
            if ( y < 10 && y >-10) 
              y = 0; 
            std::cout << "Stop command " << stopCmd << std::endl; 

            dspic.setSpSpeed(x, y,t); 

            //dspic.setVarDouble64b(CODE_VAR_SPEED_X,x);
            //dspic.setVarDouble64b(CODE_VAR_SPEED_Y,y);
            //dspic.setVarDouble64b(CODE_VAR_SPEED_T,x);
            
            std::cout << std::endl;
            buffer.clear();
            if(stopCmd == 49) 
              break; 
        }
    }

}

