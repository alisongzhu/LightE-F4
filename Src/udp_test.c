/*
 * udp_test.c
 *
 *  Created on: 2020��6��3��
 *      Author: Administrator
 */

#include "include.h"



u8 lwip_test_init(void)
{
	struct udp_pcb *pcb;
	ip_addr_t remote_ip;
	pcb	= udp_new();
	if(pcb == NULL)	 // ����ʧ��
	{
		return 1;
	}else
	{

		if(udp_bind(pcb, IP_ADDR_ANY, 8089) == ERR_OK ) // Ϊ����IP�󶨶˿ڣ�IP_ADDR_ANYΪ0����ʵ˵��ʹ�ñ���IP��ַ���Ƽ�����ʹ�á���ΪDHCP����£��������޷�����֪��IP�ġ�
		{
			udp_recv(pcb, udp_test_recv, NULL);     // ע�ᱨ�Ĵ���ص�
			printf("local_port %d\r\n", pcb->local_port);
		}else
			return 1;
	}
	return 0;
}



//UDP�������ص�����
void udp_test_recv(void *arg,struct udp_pcb *upcb,struct pbuf *p,struct ip4_addr *addr,u16_t port)
{
	u32 data_len = 0;
	struct pbuf *q;

	struct ip4_addr my_ipaddr;
	unsigned char *temp = (unsigned char *)addr;
	IP4_ADDR(&my_ipaddr, temp[0], temp[1], temp[2], temp[3]); // ����ԴIP
	udp_sendto(upcb, p, &my_ipaddr, port); // �����ķ��ظ�ԭ����
}

