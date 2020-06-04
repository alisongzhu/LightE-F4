/*
 * udp_test.c
 *
 *  Created on: 2020年6月3日
 *      Author: Administrator
 */

#include "include.h"



u8 lwip_test_init(void)
{
	struct udp_pcb *pcb;
	ip_addr_t remote_ip;
	pcb	= udp_new();
	if(pcb == NULL)	 // 申请失败
	{
		return 1;
	}else
	{
		IP4_ADDR(&remote_ip,192, 168, 1, 100);// 设置目标IP及端口
		if(udp_connect(pcb, &remote_ip, 8080) == ERR_OK ) // 连接到指定的IP地址和端口
		{
			if(udp_bind(pcb, IP_ADDR_ANY, 8080) == ERR_OK ) // 为本地IP绑定端口，IP_ADDR_ANY为0，其实说明使用本地IP地址，推荐优先使用。因为DHCP情况下，我们是无法事先知道IP的。
			{
				udp_recv(pcb, udp_test_recv, NULL); // 注册报文处理回调
				printf("local_port %d\r\n", pcb->local_port);
			}
		}else
		return 1;
	}
	return 0;
}



//UDP服务器回调函数
void udp_test_recv(void *arg,struct udp_pcb *upcb,struct pbuf *p,struct ip4_addr *addr,u16_t port)
{
	u32 data_len = 0;
	struct pbuf *q;

	struct ip4_addr my_ipaddr;
	unsigned char *temp = (unsigned char *)addr;
	IP4_ADDR(&my_ipaddr, temp[0], temp[1], temp[2], temp[3]); // 保存源IP
	udp_sendto(upcb, p, &my_ipaddr, port); // 将报文返回给原主机
}

