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
    int startServer()
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
            Sockets::dataTreatment(newClient);
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
    int dataTreatment(SOCKET newClient)
    {
        std::vector<unsigned char> buffer;

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
                    std::cout << "Error ! Problem getting the message";
                    buffer.clear();
                    return -2;
                }
                else
                {
                    receivedSize += ret;
                }
            } while ( receivedSize < expectedSize ); // We read onle the the amount of data expected

            // Prints buffer to the console
            for(int i=0; i<buffer.size(); i++)
            {	
            	if(buffer.at(i) != ';')
            		std::cout << static_cast<uint8_t>(buffer.at(i));
        		else 
        			std::cout << buffer.at(i); 
            }
            std::cout << std::endl;
            buffer.clear();
        }
    }

}

