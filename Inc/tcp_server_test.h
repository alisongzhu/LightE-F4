/*
 * tcp_server_test.h
 *
 *  Created on: 2020��6��4��
 *      Author: Administrator
 */

#ifndef TCP_SERVER_TEST_H_
#define TCP_SERVER_TEST_H_

#define     MAX_NAME_SIZE       32

struct name
{
        int     length;
        char    bytes[MAX_NAME_SIZE];
};

u8 lwip_tcp_server_test_init(void);

#endif /* TCP_SERVER_TEST_H_ */
