#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <AP_HAL/AP_HAL.h>

#include "CAN.h"

extern const AP_HAL::HAL& hal;

// https://github.com/JCube001/socketcan-demo/blob/master/src/socketcan-raw-demo.cpp

using namespace Linux;

void CAN::init() {
   struct ifreq ifr;
   struct sockaddr_can addr;

   _socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
   if(_socket < 0)
   {
       return (-1);
   }
   addr.can_family = AF_CAN;
   strcpy(ifr.ifr_name, "vcan0");
   if (ioctl(_socket, SIOCGIFINDEX, &ifr) < 0)
   {
      // TODO fail init
      return;
   }
   addr.can_ifindex = ifr.ifr_ifindex;
   fcntl(_socket, F_SETFL, O_NONBLOCK);
   if (bind(_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
   {
      // TODO fail init
      return;
   }

   return;
}

void CAN::write() {
  int retval;

  retval = write(_socket, frame, sizeof(struct can_frame));

   if (retval != sizeof(struct can_frame))
   {
       return (-1);
   }
   else
   {
       return (0);
   }
}


void CAN::read() {
  struct can_frame frame_rd;
  int recvbytes = 0;
  read_can_port = 1;
  while(read_can_port)
  {
      struct timeval timeout = {1, 0};
      fd_set readSet;
      FD_ZERO(&readSet);
      FD_SET(soc, &readSet);
      if (select((soc + 1), &readSet, NULL, NULL, &timeout) >= 0)
      {
          if (!read_can_port)
          {
              break;
          }
          if (FD_ISSET(soc, &readSet))
          {
              recvbytes = read(soc, &frame_rd, sizeof(struct can_frame));
              if(recvbytes)
              {
                  printf(“dlc = %d, data = %s\n”, frame_rd.can_dlc,frame_rd.data);
              }
          }
      }
  }
}
