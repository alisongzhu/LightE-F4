/*
 * tcp_client_test.c
 *
 *  Created on: 2020年6月4日
 *      Author: Administrator
 */

#include "include.h"

err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err);
err_t tcp_client_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *tcp_recv_pbuf, err_t err);

u8 lwip_tcp_client_init(void)
{
 	struct tcp_pcb *tcppcb;  	//定义一个TCP服务器控制块
    ip_addr_t rmtipaddr;  	//远端ip地址
	tcppcb=tcp_new();	//创建一个新的pcb
	if(tcppcb)			//创建成功
	{
		IP4_ADDR(&rmtipaddr,192,168,1,100);
		tcp_connect(tcppcb,&rmtipaddr,8000,tcp_client_connected);  //连接到目的地址的指定端口上,当连接成功后回调tcp_client_connected()函数
	}
	return 1;
}

//lwIP TCP连接建立后调用回调函数
err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err)
{
    /* 确认连接 */
    tcp_arg(tpcb, mem_calloc(sizeof(struct name), 1));

    /* 发送一个建立连接的字符串 */
    tcp_write(tpcb, "hello my dream \n\r",strlen("hello my dream \n\r  "), 1);

    /* 配置接收回调函数 */
    tcp_recv(tpcb, tcp_client_recv);

    return ERR_OK;

}


//lwIP tcp_recv()函数的回调函数
err_t tcp_client_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *tcp_recv_pbuf, err_t err)
{
	  struct pbuf *tcp_send_pbuf;
	  struct name *name = (struct name *)arg;

	if (tcp_recv_pbuf != NULL)
	{
			/* 扩大收发数据的窗口 */
			tcp_recved(pcb, tcp_recv_pbuf->tot_len);

			if (!name)
			{
					pbuf_free(tcp_recv_pbuf);
					return ERR_ARG;
			}

			/* 将接收的数据拷贝给发送结构体 */
			tcp_send_pbuf = tcp_recv_pbuf;

			/* 换行 */
			tcp_write(pcb, "\r\n", strlen("\r\n"), 1);
			/* 将接收到的数据再转发出去 */
			tcp_write(pcb, tcp_send_pbuf->payload, tcp_send_pbuf->len, 1);

			pbuf_free(tcp_recv_pbuf);
	}
	else if (err == ERR_OK)
	{
			/* 释放内存 */
			mem_free(name);
			return tcp_close(pcb);
	}

	return ERR_OK;
}
