/*
 * udp_test.h
 *
 *  Created on: 2020Äê6ÔÂ3ÈÕ
 *      Author: Administrator
 */

#ifndef UDP_TEST_H_
#define UDP_TEST_H_



u8 lwip_test_init(void);
void udp_test_recv(void *arg,struct udp_pcb *upcb,struct pbuf *p,struct ip4_addr *addr,u16_t port);
void udp_test_recv(void *arg,struct udp_pcb *upcb,struct pbuf *p,struct ip4_addr *addr,u16_t port);



#endif /* UDP_TEST_H_ */
