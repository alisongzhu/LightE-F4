/*
 * tcp_server_test.c
 *
 *  Created on: 2020��6��4��
 *      Author: Administrator
 */

#include "include.h"

err_t tcp_server_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *tcp_recv_pbuf, err_t err);
err_t tcp_server_accept(void *arg,struct tcp_pcb *newpcb,err_t err);

/***
 * �������� : Tcp_Server_Init();
 *
 * �������� : TCP��������ʼ��;
 *
 * ����ֵ   : ��;
 *
 * ����ֵ   : ��;
 *
 **/
u8 lwip_tcp_server_test_init(void)
{
     struct tcp_pcb *tcp_server_pcb;

     /* Ϊtcp����������һ��tcp_pcb�ṹ��    */
     tcp_server_pcb = tcp_new();

     /* �󶨱��ض˺ź�IP��ַ */
     tcp_bind(tcp_server_pcb, IP_ADDR_ANY, 8088);

     /* ����֮ǰ�����Ľṹ��tcp_server_pcb */
     tcp_server_pcb = tcp_listen(tcp_server_pcb);

     /* ��ʼ���ṹ����ջص����� */
     tcp_accept(tcp_server_pcb, tcp_server_accept);

	return 1;
}


/***
 * �������� : tcp_server_accept();
 *
 * �������� : lwip���ݽ��ջص�������������tcp���ӵ�ȷ�ϣ����ջص�����������;
 *
 * ����ֵ   : *arg, *pcb, err ;
 *
 * ����ֵ   : ERR_OK �޴���;
 *
 **/
err_t tcp_server_accept(void *arg,struct tcp_pcb *newpcb,err_t err)
{
    /* ȷ�ϼ��������� */
    tcp_arg(newpcb, mem_calloc(sizeof(struct name), 1));

    /* ����һ���������ӵ��ַ��� */
    tcp_write(newpcb, "hello my dream \n\r",strlen("hello my dream \n\r  "), 1);

    /* ���ý��ջص����� */
    tcp_recv(newpcb, tcp_server_recv);

    return ERR_OK;
}




/***
 * �������� : tcp_server_recv();
 *
 * �������� : ���ܵ����ݺ󣬽����ݿ���ת����ȥ;
 *
 * ����ֵ   : *arg, *pcb, *tcp_recv_pbuf, err;
 *
 * ����ֵ   : ERR_ARG �Ƿ��߼���ERR_OK�޴���;
 *
 **/
err_t tcp_server_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *tcp_recv_pbuf, err_t err)
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






