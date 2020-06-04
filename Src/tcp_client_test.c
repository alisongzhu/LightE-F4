/*
 * tcp_client_test.c
 *
 *  Created on: 2020��6��4��
 *      Author: Administrator
 */

#include "include.h"

err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err);
err_t tcp_client_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *tcp_recv_pbuf, err_t err);

u8 lwip_tcp_client_init(void)
{
 	struct tcp_pcb *tcppcb;  	//����һ��TCP���������ƿ�
    ip_addr_t rmtipaddr;  	//Զ��ip��ַ
	tcppcb=tcp_new();	//����һ���µ�pcb
	if(tcppcb)			//�����ɹ�
	{
		IP4_ADDR(&rmtipaddr,192,168,1,100);
		tcp_connect(tcppcb,&rmtipaddr,8000,tcp_client_connected);  //���ӵ�Ŀ�ĵ�ַ��ָ���˿���,�����ӳɹ���ص�tcp_client_connected()����
	}
	return 1;
}

//lwIP TCP���ӽ�������ûص�����
err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err)
{
    /* ȷ������ */
    tcp_arg(tpcb, mem_calloc(sizeof(struct name), 1));

    /* ����һ���������ӵ��ַ��� */
    tcp_write(tpcb, "hello my dream \n\r",strlen("hello my dream \n\r  "), 1);

    /* ���ý��ջص����� */
    tcp_recv(tpcb, tcp_client_recv);

    return ERR_OK;

}


//lwIP tcp_recv()�����Ļص�����
err_t tcp_client_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *tcp_recv_pbuf, err_t err)
{
	  struct pbuf *tcp_send_pbuf;
	  struct name *name = (struct name *)arg;

	if (tcp_recv_pbuf != NULL)
	{
			/* �����շ����ݵĴ��� */
			tcp_recved(pcb, tcp_recv_pbuf->tot_len);

			if (!name)
			{
					pbuf_free(tcp_recv_pbuf);
					return ERR_ARG;
			}

			/* �����յ����ݿ��������ͽṹ�� */
			tcp_send_pbuf = tcp_recv_pbuf;

			/* ���� */
			tcp_write(pcb, "\r\n", strlen("\r\n"), 1);
			/* �����յ���������ת����ȥ */
			tcp_write(pcb, tcp_send_pbuf->payload, tcp_send_pbuf->len, 1);

			pbuf_free(tcp_recv_pbuf);
	}
	else if (err == ERR_OK)
	{
			/* �ͷ��ڴ� */
			mem_free(name);
			return tcp_close(pcb);
	}

	return ERR_OK;
}
