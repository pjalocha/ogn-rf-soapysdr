#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>

#include <vector>

#include "socket.h"

// ======================================================================================================

class TCP_DataServer
{ public:
   int Server;                                 // server socket
   std::vector<int> Client;                    // list of client sockets
   static int const MaxClients = 100;          // max. number of clients, if more than close the oldest
   float SendTimeout;                          // [s] timeout for sending data

  public:
   TCP_DataServer() { BlockSIGPIPE(); Server=(-1); SendTimeout=1.0; }
  ~TCP_DataServer() { Close(); }

   void static BlockSIGPIPE(void)
   { sigset_t set;                             // disable SIGPIPE signal for the current thread when writing to a closed socket
     sigemptyset (&set);                       // this is required for the DataServer to function properly.
     sigaddset (&set, SIGPIPE);
     pthread_sigmask(SIG_BLOCK, &set, 0); }

   int Listen(int ServerPort, int MaxCons=16)        // start listening on the given port
   { Close();
     Server = socket(AF_INET, SOCK_STREAM, 0);
     if(Server<0)
     { printf("DataServer[%d]: cannot open a socket\n", ServerPort);
       return -1; }

     int Set=1;
     setsockopt(Server, SOL_SOCKET, SO_REUSEADDR, &Set, sizeof(Set));

     int Flags = fcntl(Server, F_GETFL, 0);
     Flags |=  O_NONBLOCK;
     fcntl(Server, F_SETFL, Flags);

     struct sockaddr_in ListenAddress;
     bzero((char *) &ListenAddress, sizeof(ListenAddress));
     ListenAddress.sin_family      = AF_INET;
     ListenAddress.sin_addr.s_addr = htonl(INADDR_ANY);
     ListenAddress.sin_port        = htons(ServerPort);
     if(bind(Server, (struct sockaddr *) &ListenAddress, sizeof(ListenAddress))<0) { Close(); return -1; }

     if(listen(Server, MaxCons)<0) { Close(); return -1; }

     printf("DataServer[%d]: now listen\n", Port());
     return 0; }

   int Port(void)
   { struct sockaddr_in sin;
     socklen_t len = sizeof(sin);
     if (getsockname(Server, (struct sockaddr *)&sin, &len)<0) return 0;
     return ntohs(sin.sin_port); }

   int Clients(void) const { return Client.size(); }
   // operator int [](int Idx) { return Client[Idx]; }

   int ReceiveQueue(int Idx)
   { int Bytes;
     if(ioctl(Client[Idx], FIONREAD, &Bytes)<0) return -1;
     return Bytes; }

   int Receive(char *Message, int MaxLen, int Idx)
   { if(Client[Idx]<0) return -1;
     return recv(Client[Idx], Message, MaxLen, MSG_NOSIGNAL); }

   int Accept(void)
   { int Count=0;
     if(Server<0) return Server;
     for( ; ; )
     { struct sockaddr_in Addr = { 0 }; socklen_t Len=sizeof(Addr);
       int New=accept(Server, (struct sockaddr *)&Addr, &Len);
       if(New<0)
       { if(errno!=EAGAIN) { printf("TCP_DataServer::Accept() => %s\n", strerror(errno)); Close(); }
         break; }
       int Flag = 1;
       // setsockopt(New, SOL_SOCKET, SO_NOSIGPIPE, &Flag, sizeof(Flag));
       // setsockopt(New, SOL_TCP, TCP_NODELAY, [1], 4);
       setsockopt(New, SOL_SOCKET, SO_KEEPALIVE, &Flag, sizeof(Flag));
       if(SendTimeout>0)
       { struct timeval Time;
         Time.tv_sec  = floor(SendTimeout);                                // [ sec]
         Time.tv_usec = floor(1e6*(SendTimeout-Time.tv_sec));              // [usec]
         setsockopt(New, SOL_SOCKET, SO_SNDTIMEO, &Time, sizeof(Time)); }  // if transmission to socket would take longer then we give up
       // int Flags = fcntl(New, F_GETFL, 0);
       // fcntl(New, F_SETFL, Flags | O_NONBLOCK);
       Client.push_back(New);
       printf("DataServer[%d]: accept client: %d (%d clients on the list)\n", Port(), New, Clients());
       Count++; }
     return Count; }

   void Close(size_t Idx)
   { if(Idx>=Client.size()) return;
// #if __WORDSIZE==64
     printf("DataServer[%d]: close client %d (%d clients on the list)\n", Port(), (int)Idx, Clients());
// #else
//      printf("DataServer[%d]: close client %lu (%d clients on the list)\n", Port(), Idx, Clients());
// #endif
     close(Client[Idx]);
     Client[Idx]=(-1); }
/*
   int Error(int Socket)
   { int Error = 0;
     socklen_t Len = sizeof (Error);
     if(getsockopt(Socket, SOL_SOCKET, SO_ERROR, &Error, &Len)<0) return -1;
     return Error; }
*/

   int RemoveClosed(void)
   { int Count=0;
     if(Client.size()==0) return 0;
     size_t NewIdx=0;
     for(size_t Idx=0; Idx<Client.size(); ) // go throught the list of clients
     { bool Dead = Client[Idx]<0;
       // if(!Dead)
       // { Dead = Error(Client[Idx]);
       //   if(Dead)
       //   { printf("DataServer[%d]: close client %d (%d clients on the list)\n", Port(), Client[Idx], Clients());
       //     Close(Client[Idx]); }
       // }
       if(NewIdx<Idx) Client[NewIdx]=Client[Idx];
       if(Dead) Count++;
           else NewIdx++;
       Idx++; }
     if(Count)
     { printf("DataServer[%d]: remove %d closed clients (%d clients on the list)\n", Port(), Count, Clients());
       Client.resize(NewIdx); }
     if(Client.size()>MaxClients) { close(Client[0]); Client[0]=(-1); }
     return Count; }

   void Send(char Char)
   { Send(&Char, 1); }

   void Send(const char *Message)
   { Send(Message, strlen(Message)); }

   void Send(const char *Message, int MsgLen)                        // send a message, number of bytes
   { if(Server<0) return;                                            // if Server not up then give up
     for(size_t Idx=0; Idx<Client.size(); Idx++)                     //
     { if(Client[Idx]<0) continue;
       if(send(Client[Idx], Message, MsgLen, MSG_NOSIGNAL)!=MsgLen)
       { printf("DataServer[%d]: send() error => close client %lu\n", Port(), Idx);
         Close(Idx); }
     }
   }

   void Send(const uint8_t *Message, int MsgLen) { return Send((const char *)Message, MsgLen); }

/*
   void SendOOB(const char Msg)
   { if(Server<0) return;
     for(size_t Idx=0; Idx<Client.size(); Idx++)
     { if(Client[Idx]<0) continue;
       if(send(Client[Idx], &Msg, 1, MSG_OOB)!=1)
         Close(Idx); }
   }
*/
   void Close(void)
   { if(Server<0) return;
     for(size_t Idx=0; Idx<Client.size(); Idx++)
     { if(Client[Idx]>=0) close(Client[Idx]); }
     Client.resize(0);
     close(Server); Server=(-1);
   }

   int isListenning(void) const { return Server>=0; }

} ;

// ======================================================================================================

