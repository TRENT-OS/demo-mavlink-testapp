/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <autoconf.h>
#include <camkes.h>
#include <stdio.h>
#include <virtqueue.h>
#include <camkes/virtqueue.h>
#include <utils/util.h>
#include <string.h>

#include <net/ethernet.h>
#include <netinet/ether.h>
#include <netinet/in.h>
#include <netinet/ip_icmp.h>
#include <arpa/inet.h>
#include <netinet/udp.h>

//#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINT(fmt, args...) printf(fmt, ## args)
#else
#define DEBUG_PRINT(fmt, args...)
#endif



#define ICMP_MSG_SIZE 64 - sizeof(struct icmphdr)
#define IPV4_LENGTH 4

#define LINUX_UDPPORT 10000
#define PINGCLIENT_UDPPORT 10000
#define LINUX_IP_ADDR "192.168.1.1"
#define PINGCLIENT_IP_ADDR "192.168.1.2"
#define LINUX_MAC_ADDR "02:00:00:00:AA:01"
#define PINGCLIENT_MAC_ADDR "02:00:00:00:AA:02"

virtqueue_device_t recv_virtqueue;
virtqueue_driver_t send_virtqueue;

void handle_recv_callback(virtqueue_device_t *vq);
void handle_send_callback(virtqueue_driver_t *vq);

unsigned short one_comp_checksum(char *data, size_t length)
{
    unsigned int sum = 0;
    int i = 0;

    for (i = 0; i < length - 1; i += 2) {
        unsigned short *data_word = (unsigned short *)&data[i];
        sum += *data_word;
    }
    /* Odd size */
    if (length % 2) {
        unsigned short data_word = (unsigned char)data[i];
        sum += data_word;
    }

    sum = (sum >> 16) + (sum & 0xFFFF);
    sum += (sum >> 16);
    return ~sum;
}

int send_outgoing_packet(char *outgoing_data, size_t outgoing_data_size)
{
    void *buf = NULL;
    int err = camkes_virtqueue_buffer_alloc(&send_virtqueue, &buf, outgoing_data_size);
    if (err) {
        return -1;
    }
    memcpy(buf, outgoing_data, outgoing_data_size);
    if (camkes_virtqueue_driver_send_buffer(&send_virtqueue, buf, outgoing_data_size) != 0) {
        camkes_virtqueue_buffer_free(&send_virtqueue, buf);
        return -1;
    }
    send_virtqueue.notify();
    return 0;
}

void print_ip_packet(void *ip_buf, size_t ip_length)
{
    struct iphdr *ip = ip_buf;
    unsigned char *ip_packet = (unsigned char *)ip_buf;

    printf("Packet Contents:");
    for (int i = 0; i < ip_length; i++) {
        if (i % 15 == 0) {
            printf("\n%d:\t", i);
        }
        printf("%x ", ip_packet[i]);
    }
    printf("\n");

    struct in_addr saddr = {ip->saddr};
    struct in_addr daddr = {ip->daddr};
    printf("IP Header - Version: IPv%d protocol: %d | src address: %s",
           ip->version, ip->protocol, inet_ntoa(saddr));
    printf(" | dest address: %s\n", inet_ntoa(daddr));
    if (ip->protocol == 1)
    {
        struct icmphdr *icmp = ip_buf + sizeof(struct iphdr);
        printf("ICMP Header - Type: %d | id: %d | seq: %d\n",
               icmp->type, icmp->un.echo.id, icmp->un.echo.sequence);
    }
    if (ip->protocol == 17)
    {
        struct udphdr *udp = ip_buf + sizeof(struct iphdr);
        printf("UDP Header - Sourceport: %d | Destinationport: %d \n", ntohs(udp->uh_sport), ntohs(udp->uh_dport));
    }
    printf("\n");
}

int create_arp_req_reply(char *recv_data, unsigned int recv_data_size)
{
    char reply_buffer[ETHERMTU];
    static uint8_t first_run=0;
    //---------------------------------
    //| ethhdr | ether_arp            |
    //---------------------------------
    struct ether_arp *arp_req = (struct ether_arp *)(recv_data + sizeof(struct ethhdr));

    struct ethhdr *send_reply = (struct ethhdr *) reply_buffer;
    struct ether_arp *arp_reply = (struct ether_arp *)(reply_buffer + sizeof(struct ethhdr));

    if(first_run == 0)
    {
        pingready_emit();
        first_run = 1;
    }
    memcpy(send_reply->h_dest, arp_req->arp_sha, ETH_ALEN);
    send_reply->h_proto = htons(ETH_P_ARP);

    /* MAC Address */
    memcpy(arp_reply->arp_tha, arp_req->arp_sha, ETH_ALEN);
    memcpy(arp_reply->arp_sha, arp_req->arp_sha, ETH_ALEN);
    arp_reply->arp_sha[5] = arp_reply->arp_sha[5] + 2;

    memcpy(send_reply->h_source, arp_reply->arp_sha, ETH_ALEN);
    /* IP Addresss */
    for (int i = 0; i < IPV4_LENGTH; i++) {
        arp_reply->arp_spa[i] = arp_req->arp_tpa[i];
    }
    for (int i = 0; i < IPV4_LENGTH; i++) {
        arp_reply->arp_tpa[i] = arp_req->arp_spa[i];
    }
    /* ARP header fields */
    arp_reply->ea_hdr.ar_hrd = htons(ARPHRD_ETHER);
    arp_reply->ea_hdr.ar_pro = htons(ETH_P_IP);
    arp_reply->ea_hdr.ar_op = htons(ARPOP_REPLY);
    arp_reply->ea_hdr.ar_hln = ETH_ALEN;
    arp_reply->ea_hdr.ar_pln = IPV4_LENGTH;

    return send_outgoing_packet(reply_buffer, sizeof(struct ethhdr) + sizeof(struct ether_arp));
}

int send_udp_packet(void)
{
    char buffer[ETHERMTU];
    DEBUG_PRINT("PINGCLIENT prepar UDP package\n");
    struct ethhdr *eth_req = (struct ethhdr *) buffer;
    struct iphdr *ip_req = (struct iphdr *)(buffer + sizeof(struct ethhdr));
    struct udphdr *udp_req = (struct udphdr *)(buffer + sizeof(struct ethhdr) + sizeof(struct iphdr));
    char *packet = (buffer + sizeof(struct ethhdr) + sizeof(struct iphdr) + sizeof(struct udphdr));
    // Convert mac from string to bytes
    struct ether_addr linux_mac;
    struct ether_addr pingclient_mac;
    ether_aton_r(LINUX_MAC_ADDR, &linux_mac);
    ether_aton_r(PINGCLIENT_MAC_ADDR, &pingclient_mac);

    char linux_ip[] = LINUX_IP_ADDR;
    char pingclient_ip[] = PINGCLIENT_IP_ADDR;
    //fill in default information
    ip_req->version = 4;
    ip_req->ihl = 5; // length of header (ihl * 32 bits)
    ip_req->tos = 0; // type of service field
    ip_req->id = 0;
    ip_req->frag_off = 0;
    ip_req->ttl = 3;
    // Specify IP UDP
    ip_req->protocol = 17;
    eth_req->h_proto = htons(ETH_P_IP);
    //Don't care about the checksum
    udp_req->uh_sum = 0;

    // Fill in information about linux destination
    udp_req->uh_dport = htons(LINUX_UDPPORT);
    ip_req->daddr = inet_addr(linux_ip);

    // Fill in infromation about pingclient source
    udp_req->uh_sport = htons(PINGCLIENT_UDPPORT);
    ip_req->saddr = inet_addr(pingclient_ip);
    DEBUG_PRINT("PINGCLIENT source max: ");
    for(int i = 0; i < ETH_ALEN; i++)
    {
        DEBUG_PRINT("%x:", pingclient_mac.ether_addr_octet[i]);
        eth_req->h_source[i] = pingclient_mac.ether_addr_octet[i];
        eth_req->h_dest[i] = linux_mac.ether_addr_octet[i];
    }
    DEBUG_PRINT("\n");
    for(int i = 0; i < 10; i++)
    {
        *(packet + i) = i;
    }
    udp_req->uh_ulen = htons(sizeof(struct udphdr) + 10);
    DEBUG_PRINT("upd len: %d", ntohs(udp_req->uh_ulen));
    ip_req->tot_len = htons(sizeof(struct iphdr) + sizeof(struct udphdr) + 10);
    DEBUG_PRINT("ip len: %d", ntohs(ip_req->tot_len));
    /* Need to set checksum to 0 before calculating checksum of the header */
    ip_req->check = 0;
    ip_req->check = one_comp_checksum((char *)ip_req, sizeof(struct iphdr));
#ifdef DEBUG
    print_ip_packet((char *)ip_req, ntohs(ip_req->tot_len));
#endif
    DEBUG_PRINT("PINGCLIENT send UDP package\n");
    DEBUG_PRINT("Packet raw hex:");
    for(int i = 0; i < sizeof(struct ethhdr) + sizeof(struct iphdr) + sizeof(struct udphdr) + 10; i++)
    {
        DEBUG_PRINT(" %02x", buffer[i]);
    }
    DEBUG_PRINT("\n");
    return send_outgoing_packet(buffer,
                                sizeof(struct ethhdr) + sizeof(struct iphdr) + sizeof(struct udphdr) + 10);
}

int create_icmp_req_reply(char *recv_data, unsigned int recv_data_size)
{

    struct ethhdr *eth_req = (struct ethhdr *) recv_data;
    struct iphdr *ip_req = (struct iphdr *)(recv_data + sizeof(struct ethhdr));
    struct icmphdr *icmp_req = (struct icmphdr *)(recv_data + sizeof(struct ethhdr) + sizeof(struct iphdr));

    char reply_buffer[ETHERMTU];
    struct ethhdr *eth_reply = (struct ethhdr *) reply_buffer;
    struct iphdr *ip_reply = (struct iphdr *)(reply_buffer + sizeof(struct ethhdr));
    struct icmphdr *icmp_reply = (struct icmphdr *)(reply_buffer + sizeof(struct ethhdr) + sizeof(struct iphdr));
    char *icmp_msg = (char *)(icmp_reply + 1);

    memcpy(eth_reply->h_dest, eth_req->h_source, ETH_ALEN);
    memcpy(eth_reply->h_source, eth_req->h_dest, ETH_ALEN);
    eth_reply->h_proto = htons(ETH_P_IP);

    memcpy(ip_reply, ip_req, sizeof(struct iphdr));
    in_addr_t saddr = ip_reply->saddr;
    ip_reply->saddr = ip_reply->daddr;
    ip_reply->daddr = saddr;

    memset(icmp_msg, 0, ICMP_MSG_SIZE);
    icmp_reply->un.echo.sequence =  icmp_req->un.echo.sequence;
    icmp_reply->un.echo.id = icmp_req->un.echo.id;
    icmp_reply->type = ICMP_ECHOREPLY;
    icmp_reply->checksum = one_comp_checksum((char *)icmp_reply, sizeof(struct icmphdr) + ICMP_MSG_SIZE);

    /* Need to set checksum to 0 before calculating checksum of the header */
    ip_reply->check = 0;
    ip_reply->check = one_comp_checksum((char *)ip_reply, sizeof(struct iphdr));

    return send_outgoing_packet(reply_buffer,
                                sizeof(struct ethhdr) + sizeof(struct iphdr) + sizeof(struct icmphdr) + ICMP_MSG_SIZE);
}

void handle_recv_data(char *recv_data, unsigned int recv_data_size)
{
    struct ethhdr *rcv_req = (struct ethhdr *) recv_data;
    if (ntohs(rcv_req->h_proto) == ETH_P_ARP) {
        create_arp_req_reply(recv_data, recv_data_size);
    } else if (ntohs(rcv_req->h_proto) == ETH_P_IP) {
        char ip_packet[ETHERMTU];
        memcpy(ip_packet, recv_data + sizeof(struct ethhdr), recv_data_size - sizeof(struct ethhdr));
        print_ip_packet(ip_packet, recv_data_size - sizeof(struct ethhdr));
        create_icmp_req_reply(recv_data, recv_data_size);;
    }

}

void handle_recv_callback(virtqueue_device_t *vq)
{
    void *buf = NULL;
    unsigned int buf_size = 0;
    vq_flags_t flag;
    virtqueue_ring_object_t handle;
    if (!virtqueue_get_available_buf(vq, &handle)) {
        ZF_LOGE("Client virtqueue dequeue failed");
        return;
    }

    while (camkes_virtqueue_device_gather_buffer(vq, &handle, &buf, &buf_size, &flag) >= 0) {
        handle_recv_data((char *) buf, buf_size);
    }

    if (!virtqueue_add_used_buf(&recv_virtqueue, &handle, 0)) {
        ZF_LOGE("Unable to enqueue used recv buffer");
        return;
    }

    recv_virtqueue.notify();
}

void handle_send_callback(virtqueue_driver_t *vq)
{
    void *buf = NULL;
    unsigned int buf_size = 0;
    uint32_t wr_len = 0;
    vq_flags_t flag;
    virtqueue_ring_object_t handle;
    if (!virtqueue_get_used_buf(vq, &handle,(uint32_t *) &wr_len)) {
        ZF_LOGE("Client virtqueue dequeue failed");
        return;
    }

    while (camkes_virtqueue_driver_gather_buffer(vq, &handle, &buf, &buf_size, &flag) >= 0) {
        /* Clean up and free the buffer we allocated */
        camkes_virtqueue_buffer_free(vq, buf);
    }
}


void ping_wait_callback(void)
{
    if (VQ_DEV_POLL(&recv_virtqueue)) {
        handle_recv_callback(&recv_virtqueue);
    }

    if (VQ_DRV_POLL(&send_virtqueue)) {
        handle_send_callback(&send_virtqueue);
    }
}

int run(void)
{
    ZF_LOGE("Starting ping echo component");

    /* Initialise recv virtqueue */
    int err = camkes_virtqueue_device_init(&recv_virtqueue, 0);
    if (err) {
        ZF_LOGE("Unable to initialise recv virtqueue");
        return 1;
    }

    /* Initialise send virtqueue */
    err = camkes_virtqueue_driver_init(&send_virtqueue, 1);
    if (err) {
        ZF_LOGE("Unable to initialise send virtqueue");
        return 1;
    }

    return 0;
}
