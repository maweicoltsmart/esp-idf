#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "param.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include "mydns.h"

static void
dns_found(const char *name, const ip_addr_t *addr, void *arg)
{
  LWIP_UNUSED_ARG(arg);
  printf("%s: %s\n", name, addr ? ipaddr_ntoa(addr) : "<not found>");
  if(addr)
  {
    stWork_Pragma.stRemote_IP.remoteip.u_addr.ip4 = addr->u_addr.ip4;
  }
  else
  {
    ip4_addr_set_u32(&stWork_Pragma.stRemote_IP.remoteip.u_addr.ip4, IPADDR_ANY);
  }
}

void get_remote_ip(void)
{
    if(ip4addr_aton((char const*)stWork_Pragma.stRemote_IP.hostname, &stWork_Pragma.stRemote_IP.remoteip.u_addr.ip4) == 0)
    {
        printf("dns server: %s\r\n", ipaddr_ntoa(dns_getserver(1)));
        if (dns_gethostbyname((char const*)stWork_Pragma.stRemote_IP.hostname,
            &stWork_Pragma.stRemote_IP.remoteip, dns_found, 0) == ERR_OK) {
            dns_found((char const*)stWork_Pragma.stRemote_IP.hostname, &stWork_Pragma.stRemote_IP.remoteip, 0);
        }
    }
}

TickType_t GetCurrentTime(void)
{
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}
TickType_t TimerGetElapsedTime( TickType_t savedTime )
{
    volatile TickType_t elapsedTime = 0;

    // Needed at boot, cannot compute with 0 or elapsed time will be equal to current time
    /*if( savedTime == 0 )
    {
        return 0;
    }*/

    elapsedTime = GetCurrentTime();

    if( elapsedTime < savedTime )
    { // roll over of the counter
        return( elapsedTime + ( 0xFFFFFFFF - savedTime ) );
    }
    else
    {
        return( elapsedTime - savedTime );
    }
}