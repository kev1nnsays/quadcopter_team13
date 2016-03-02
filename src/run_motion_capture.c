// run_motion_capture.c thread
//
// Start by including Optitrack code (PacketClient.cpp) with
// main() function commented out.  Pertinent main() commands pasted into
// top-level thread function run_motion_capture in this file.
//

#define EXTERN extern
#include "../include/quadcopter_main.h"
#include "./PacketClient.cpp"

// Students: Comment the following definition out unless you're testing our geofencing software
//#define GEOFENCE_AXES

void *run_motion_capture(void *userdata){
  struct optitrack_message optmsg;
  struct motion_capture_obs mc; // Local copy of mcap_obs object (to update)
  puts("Running MOCAP..");
  // Open motion capture data file
  mcap_txt = fopen("mcap.txt","a");

  // Socket code is from the PacketClient.cpp (Optitrack) main() function
  int retval;
  char szMyIPAddress[128] = "192.168.1.123";
  char szServerIPAddress[128] = "192.168.1.123";
  in_addr MyAddress, MultiCastAddress;
  int optval = 0x100000;
  socklen_t optval_size = 4;
  struct ifaddrs *ifAddrStruct=NULL, *ifa=NULL;
  void *tmpAddrPtr=NULL;
  
  // Thread variables (original:  windows thread handlers)
  pthread_t command_listen_thread;
  pthread_t data_listen_thread;

  // server address
  /* ADD HARDCODED SERVER ADDRESS
  if(argc>1) {
    strcpy(szServerIPAddress, argv[1]);	// specified on command line
  } else {
  
    getifaddrs(&ifAddrStruct);
    for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {
      if (!ifa->ifa_addr) continue;
      if (ifa->ifa_addr->sa_family == AF_INET) { // IP4
	tmpAddrPtr = &((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
	// char addressBuffer[INET_ADDRSTRLEN];
	inet_ntop(AF_INET, tmpAddrPtr, szServerIPAddress, INET_ADDRSTRLEN);
	printf("%s IP Address %s\n", ifa->ifa_name, szServerIPAddress); 
      }
    }
    } */
  ServerAddress.s_addr = (uint32_t) inet_addr(szServerIPAddress);
  
  // client address
  /*  if(argc>2) {
    strcpy(szMyIPAddress, argv[2]);	// specified on command line
  } else {

    getifaddrs(&ifAddrStruct);
    for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {
      if (!ifa->ifa_addr) continue;
      if (ifa->ifa_addr->sa_family == AF_INET) { // IP4
	tmpAddrPtr = &((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
	// char addressBuffer[INET_ADDRSTRLEN];
	inet_ntop(AF_INET, tmpAddrPtr, szMyIPAddress, INET_ADDRSTRLEN);
	printf("%s IP Address %s\n", ifa->ifa_name, szMyIPAddress); 
      }
    }
    } */
  MyAddress.s_addr = inet_addr(szMyIPAddress);
  
  MultiCastAddress.s_addr = inet_addr(MULTICAST_ADDRESS);   
  printf("Client: %s\n", szMyIPAddress);
  printf("Server: %s\n", szServerIPAddress);
  printf("Multicast Group: %s\n", MULTICAST_ADDRESS);
  
  // create "Command" socket
  int port = 0;

  CommandSocket = CreateCommandSocket(MyAddress.s_addr,port);
  if(CommandSocket != -1) {
    // [optional] set to non-blocking
    //u_long iMode=1;
    //ioctlsocket(CommandSocket,FIONBIO,&iMode); 
    // set buffer
    setsockopt(CommandSocket, SOL_SOCKET, SO_RCVBUF, (char *)&optval, 4);
    getsockopt(CommandSocket, SOL_SOCKET, SO_RCVBUF, (char *)&optval, &optval_size);
    if (optval != 0x100000) {
      // err - actual size...
    }
    // startup our "Command Listener" thread
    pthread_create(&command_listen_thread, NULL, CommandListenThread, NULL);       
  }
  
  // create a "Data" socket
  DataSocket = socket(AF_INET, SOCK_DGRAM, 0);
  
  // allow multiple clients on same machine to use address/port
  int value = 1;
  retval = setsockopt(DataSocket, SOL_SOCKET, SO_REUSEADDR, (char*)&value, sizeof(value));
  if (retval < 0)
    {
      //close(DataSocket);
      return NULL;
    }
  
  struct sockaddr_in MySocketAddr;
  memset(&MySocketAddr, 0, sizeof(MySocketAddr));
  MySocketAddr.sin_family = AF_INET;
  MySocketAddr.sin_port = htons(PORT_DATA);
  MySocketAddr.sin_addr.s_addr = INADDR_ANY; 

  if (bind(DataSocket, (struct sockaddr *)&MySocketAddr, sizeof(struct sockaddr)) < 0)
    {
      printf("[PacketClient] bind failed.\n");
      return NULL;
    }
  // join multicast group
  struct ip_mreq Mreq;
  Mreq.imr_multiaddr = MultiCastAddress;
  Mreq.imr_interface = MyAddress;
  retval = setsockopt(DataSocket, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&Mreq, sizeof(Mreq));
  if (retval < 0)
    {
      printf("[PacketClient] join failed.\n");
      return NULL;
    }
  
  // create a 1MB buffer
  setsockopt(DataSocket, SOL_SOCKET, SO_RCVBUF, (char *)&optval, 4);
  getsockopt(DataSocket, SOL_SOCKET, SO_RCVBUF, (char *)&optval, &optval_size);
  if (optval != 0x100000)
    {
      printf("[PacketClient] ReceiveBuffer size = %d", optval);
    }
  
  // Code from DataListenThread function in PacketClient.cpp
  char  szData[20000];
  socklen_t addr_len = sizeof(struct sockaddr);
  sockaddr_in TheirAddress;
  
  // MAIN THREAD LOOP
  while (1) {

    // puts("Trying to read..");
    // Block until we receive a datagram from the network
    int nDataBytesReceived = recvfrom(DataSocket, szData, sizeof(szData), 
				      0, (sockaddr *)&TheirAddress, &addr_len);
    Unpack_to_code(szData, &optmsg);
        
    mc.time = ((double)utime_now())/1000000;

    double euler[3];
    double q[4] = {(double)optmsg.qw,(double)optmsg.qx,(double)optmsg.qy,(double)optmsg.qz};
    quaternion2euler(q, euler);

    // Mia's version
#ifdef GEOFENCE_AXES
    mc.pose[0] = (double)optmsg.z;
    mc.pose[1] = -(double)optmsg.x;
    mc.pose[2] = (double)optmsg.y;
    mc.pose[3] = euler[2];
    mc.pose[4] = -euler[0];
    mc.pose[5] = euler[1];
    
#else
    // Aircraft coordinate version
    // Optitrack coordinates:  (x "forward" (matches aircraft +x), 
    // y "up" (matches aircraft -z), z "right" (matches aircraft +y)
    // puts("Copying..");
    mc.pose[0] = (double)optmsg.x;
    mc.pose[1] = (double)optmsg.z;
    mc.pose[2] = -(double)optmsg.y;
    mc.pose[3] = euler[0];
    mc.pose[4] = euler[2];
    mc.pose[5] = -euler[1];
#endif    
    

    fprintf(mcap_txt,"%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",mc.time,
      mc.pose[0],mc.pose[1],mc.pose[2],mc.pose[3],mc.pose[4],mc.pose[5]);
    fflush(mcap_txt);
     //printf("%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",mc.time,
     // mc.pose[0],mc.pose[1],mc.pose[2],mc.pose[3],mc.pose[4],mc.pose[5]);
    
    // Update global motion capture observations
    // Older:  (mcap_obs+1), Newer:  (mcap_obs)
    pthread_mutex_lock(&mcap_mutex);
    memcpy(mcap_obs+1, mcap_obs, sizeof(struct motion_capture_obs));
    memcpy(mcap_obs, &mc, sizeof(struct motion_capture_obs));
    pthread_mutex_unlock(&mcap_mutex);

  }

  fclose(mcap_txt);
  return NULL;
}
