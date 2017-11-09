//
//  main.c
//  GPSHub
//
//  Created by road on 7/11/16.
//  Copyright (c) 2016 road. All rights reserved.
//
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <event.h>
#include <event2/listener.h>
#include <event2/bufferevent.h>
#include <event2/thread.h>

#include <pthread.h>

#include <time.h>

#include "gps_hub.h"
#include "cJSON.h"
#include "cir_queue.h"

void listener_cb(struct evconnlistener *listener, evutil_socket_t fd,
                 struct sockaddr *sock, int socklen, void *arg);

void socket_read_cb(struct bufferevent *bev, void *arg);

void socket_event_cb(struct bufferevent *bev, short events, void *arg);

static JT808Type getGPSData(unsigned char *,int);

static int unEscape (unsigned char *,unsigned char *,int);

static unsigned char checkXor(unsigned char *, int, int);

static void toHex(unsigned char , unsigned char [], int);

static char toHexLow(unsigned char);

static int isPackage(short messageBodyProperty);

static void getVehicleSimNr(unsigned char [],unsigned char []);

static float getLng(unsigned char []);

static float getLat(unsigned char []);

static short getSpeed(unsigned char []);

static short getDirection(unsigned char []);

static short getHight(unsigned char []);

static void getGpsTime(unsigned char [],unsigned char []);

static int getAcc(unsigned char []);

void * thread_queue(void * arg);

void error_handler(OCI_Error *err);

cir_queue_t g_sending_queue;

struct bufferevent * g_bev;

static otext temp[10];

int main(int argc, const char * argv[]) {

    time_t timep;
    time (&timep);

    if (!OCI_Initialize(error_handler, NULL, OCI_ENV_CONTEXT))
        return EXIT_FAILURE;
    
    OCI_Pool* pool = OCI_PoolCreate("GXYSDB", "mtrack", "mtrack", OCI_POOL_SESSION, OCI_SESSION_DEFAULT, 5, 20, 2);
    
    int res;
    
    pthread_t send_thread;
    
    init_cir_queue(&g_sending_queue);
    
    res = pthread_create(&send_thread, NULL, thread_queue, (void*)&g_sending_queue);
    
    if (res != 0){
        
        perror("Thread creation failed.");
        
        exit(EXIT_FAILURE);
        
    }

    printf("%s sending thread started...\n",asctime(gmtime(&timep)));
    
    struct st_lsnr_context * plct = (struct st_lsnr_context *)malloc(sizeof(struct st_lsnr_context));
    
    plct->pool = pool;
    
    struct sockaddr_in sin;
    memset(&sin, 0, sizeof(struct sockaddr_in));
    sin.sin_family = AF_INET;
    sin.sin_port = htons(9020);
    
    struct event_base *base = event_base_new();
    
    plct->base = base;
    
    struct evconnlistener *listener = evconnlistener_new_bind(base, listener_cb, plct,
                                                              LEV_OPT_REUSEABLE|LEV_OPT_CLOSE_ON_FREE,
                                                              -1, (struct sockaddr*)&sin,
                                                              sizeof(struct sockaddr_in));
    
    event_base_dispatch(base);
    
    pthread_join(send_thread,NULL);
    
    evconnlistener_free(listener);
    
    event_base_free(base);
    
    OCI_Cleanup();
    
    return 0;
}

void listener_cb(struct evconnlistener *listener, evutil_socket_t fd,
                 struct sockaddr *sock, int socklen, void *arg) {

    time_t timep;
    time (&timep);
    
    printf("%s accept a client %d\n", asctime(gmtime(&timep)), fd);
    
    struct st_lsnr_context * plct = (struct st_lsnr_context *)arg;
    
    struct event_base *base = plct->base;
    
    struct bufferevent *bev =  bufferevent_socket_new(base, fd,
                                                      BEV_OPT_CLOSE_ON_FREE);
    
    struct evbuffer * buf = evbuffer_new();
    
    struct st_read_context * prct = (struct st_read_context *)malloc(sizeof(struct st_read_context));
    
    prct->ctx = plct;
    
    prct->buf = buf;
    
    prct->isSendingSocket = 0;
    
    bufferevent_setcb(bev, socket_read_cb, NULL, socket_event_cb, prct);
    
    bufferevent_enable(bev, EV_READ | EV_PERSIST);
}

void socket_read_cb(struct bufferevent *bev, void *arg) {

    const char ch = 0x7e;
    
    struct st_read_context * prct = (struct st_read_context *) arg;
    
    struct evbuffer * mybuffer = prct->buf;
    
    OCI_Connection * conn = OCI_PoolGetConnection(prct->ctx->pool , NULL);
    
    OCI_Statement  *st;
    
    st = OCI_StatementCreate(conn);
    
    struct evbuffer* input = bufferevent_get_input(bev);
    size_t len = 0;
    len = evbuffer_get_length(input);
    
    unsigned char* buf;
    buf = (unsigned char*)malloc(sizeof(unsigned char)*len);
    if(NULL==buf){return;}
    
    evbuffer_remove(input,buf,len);
    
    evbuffer_add(mybuffer, buf, len);
    
    struct evbuffer_ptr result;
    
    do {
        evbuffer_ptr_set(mybuffer,&result,0,EVBUFFER_PTR_SET);
        result = evbuffer_search(mybuffer, &ch, 1, &result);
        
        buf = (unsigned char*)realloc(buf,sizeof(unsigned char)*(result.pos+1));
        evbuffer_remove(mybuffer, buf , result.pos+1);
        
        int buf_length = (int)(result.pos+1);
        
        if (buf_length >= 3) {
            
            if ((*buf == 0x02) && (*(buf+1) == 0x00)) {
                
                JT808Type jt808 = getGPSData(buf , buf_length-1);

                /******************************************************************************
                printf("%s\n", jt808._sim);
                printf("%f\n", jt808._lng); 
                printf("%f\n", jt808._lat);
                printf("%s\n", jt808._gpsTime); 
                printf("%d\n", jt808._acc);
                printf("%d\n", jt808._speed);
                printf("%d\n", jt808._direction);
               
                OCI_Prepare(st, OTEXT("insert into gps_message(GPS_DATA) values(:gps_data)"));

                OCI_BindRaw(st, ":gps_data", buf ,buf_length);

                OCI_Execute(st);
                ******************************************************************************/
                osprintf(temp, 10, OTEXT("..."));
                
                OCI_Prepare(st, OTEXT("begin ")
                            OTEXT("  P_SAVE_TRACK_NEW(:location,:lat,:lng,:speed,:direction,:status,:simno,:gpsTime); ")
                            OTEXT("end; "));
                
                OCI_BindString(st, ":location", (otext*) temp, 10);
                OCI_BindFloat(st, ":lat", &jt808._lat);
                OCI_BindFloat(st, ":lng", &jt808._lng);
                OCI_BindShort(st, ":speed", &(jt808._speed));
                OCI_BindShort(st, ":direction", &(jt808._direction));
                OCI_BindInt(st, ":status", &jt808._acc);
                OCI_BindString(st, ":simno", jt808._sim, 13);
                OCI_BindString(st, ":gpsTime", jt808._gpsTime, 13);
                OCI_Execute(st);
                
                if (g_bev){
                    
                    push_cir_queue(&g_sending_queue,jt808);
                    
                }
            }else if ((*buf == 0x20) && (*(buf+1) == 0x21)) {
                
                g_bev = bev;
                prct->isSendingSocket = 1;
                
            }
        }
    } while (result.pos >= 0);
    
    OCI_Commit(conn);
    
    OCI_ConnectionFree(conn);
    
    free(buf);
    
    buf = NULL;
    
    return;
    
}

void socket_event_cb(struct bufferevent *bev, short events, void *arg) {
    
    time_t timep;
    time (&timep);

    struct st_read_context * prct = (struct st_read_context *) arg;
    
    struct evbuffer * mybuffer = prct->buf;
    
    if (events & BEV_EVENT_EOF){
        
        if (prct->isSendingSocket){
            
            g_bev = NULL;
            clear_cir_queue(&g_sending_queue);
            prct->isSendingSocket = 0;
        }
        
        printf("%s connection closed\n",asctime(gmtime(&timep)));
        
    }
    else if (events & BEV_EVENT_ERROR){

        if (prct->isSendingSocket){
            
            g_bev = NULL;
            clear_cir_queue(&g_sending_queue);
            prct->isSendingSocket = 0;
        }
        printf("%s some other error\n",asctime(gmtime(&timep)));
    }

    evbuffer_free(mybuffer);
        
    free(prct);    
    
    bufferevent_free(bev);
    
}

static JT808Type getGPSData(unsigned char * buf, int len){
    
    JT808Type jt808;
    
    unsigned char * escData = (unsigned char*)malloc(sizeof(unsigned char)*len);
    
    unsigned char * bufferdata = (unsigned char*)malloc(sizeof(unsigned char)*len);
    
    int act_len = unEscape(buf,escData,len);
    
    unsigned char xor = checkXor(escData,1,act_len-1);
    
    if( xor == escData[act_len-1] ){
        
        jt808._messageBodyProperty = (short) ((escData[2] << 8) + escData[3]);
        
        if (!isPackage(jt808._messageBodyProperty)){
            memcpy(bufferdata, escData+12, act_len - 12);
            getVehicleSimNr(jt808._sim,escData);
            jt808._lng = getLng(bufferdata);
            jt808._lat = getLat(bufferdata);
            jt808._speed = getSpeed(bufferdata);
            jt808._direction = getDirection(bufferdata);
            getGpsTime(jt808._gpsTime,bufferdata);
            jt808._acc = getAcc(bufferdata);
        }
    }
    
    jt808._len = act_len;
    
    return jt808;
    
}

static int unEscape (unsigned char * src,unsigned char *des,int len){
    
    int i,j;
    j=0;
    for (i = 0; i < len; i++) {
        
        if (src[i] == 0x7d) {
            if (src[i + 1] == 0x01) {
                des[j++] = 0x7d;
                i++;
            } else if (src[i + 1] == 0x02) {
                des[j++] = 0x7e;
                i++;
            }
            
        } else {
            des[j++] = src[i];
        }
    }
    return j;
}

static unsigned char checkXor(unsigned char * data, int pos, int len) {
    
    unsigned char X = data[0];
    int i;
    for (i = pos; i < len; i++) {
        X = X ^ data[i];
    }
    return X;
    
}

static void toHex(unsigned char b, unsigned char buf[], int start) {
    
    unsigned char factor = 16;
    int v = b & 0xff;
    unsigned char high = (unsigned char) (v / factor);
    unsigned char low = (unsigned char) (v % factor);
    buf[start] = toHexLow(high);
    buf[start+1] = toHexLow(low);
    return;
    
}

static char toHexLow(unsigned char b) {
    
    if (b < 10) {
        return (char) ('0' + (char) b);
    } else {
        return (char) ('A' + (b - 10));
    }
}

static int isPackage(short messageBodyProperty) {
    
    return (messageBodyProperty & 0x2000) == 0x2000;
}

static void getVehicleSimNr(unsigned char buf[],unsigned char escData[]) {
    
    toHex(escData[4],buf,0);
    toHex(escData[5],buf,2);
    toHex(escData[6],buf,4);
    toHex(escData[7],buf,6);
    toHex(escData[8],buf,8);
    toHex(escData[9],buf,10);
    
    buf[12] = '\0';
    
    return;
}

static float getLng(unsigned char bufferdata[]) {
    
    return (float)((bufferdata[12] << 24)|
                   (bufferdata[13] << 16)|
                   (bufferdata[14] << 8)|
                   bufferdata[15]) * 0.000001;
}

static float getLat(unsigned char bufferdata[]) {
    
    return (float)((bufferdata[8] << 24) |
                   (bufferdata[9] << 16) |
                   (bufferdata[10] << 8) |
                   bufferdata[11]) * 0.000001;
}

static short getSpeed(unsigned char bufferdata[]) {
    
    return (short)((bufferdata[18] << 8) + bufferdata[19]);
    
}

static short getDirection(unsigned char bufferdata[]) {
    
    return (short)((bufferdata[20] << 8) + bufferdata[21]);
    
}

static short getHight(unsigned char bufferdata[]) {
    
    return (short)((bufferdata[16] << 8) + bufferdata[17]);
}

static void getGpsTime(unsigned char buf[],unsigned char bufferdata[]) {
    
    toHex(bufferdata[22],buf,0);
    toHex(bufferdata[23],buf,2);
    toHex(bufferdata[24],buf,4);
    toHex(bufferdata[25],buf,6);
    toHex(bufferdata[26],buf,8);
    toHex(bufferdata[27],buf,10);
    buf[12] = '\0';
    return;
}

static int getAcc(unsigned char bufferdata[]) {
    
    unsigned long result = (unsigned long) ((bufferdata[4] << 24) + (bufferdata[5] << 16) + (bufferdata[6] << 8) + (bufferdata[7]));
    
    return (result & 0x10000000 ) == 0x10000000;
    
}

void *thread_queue(void *cirqueue)

{
    
    JT808Type jt808;
    cJSON *root;
    
    while (isRunning((cir_queue_t*)cirqueue)){
        
        jt808 = pop_cir_queue((cir_queue_t*)cirqueue);
        
        
        if (g_bev) {
            
            root=cJSON_CreateObject();
            
            cJSON_AddStringToObject(root,"sim",(char *)jt808._sim);
            cJSON_AddNumberToObject(root,"lng",jt808._lng);
            cJSON_AddNumberToObject(root,"lat",jt808._lat);
            cJSON_AddStringToObject(root,"time",(char *)jt808._gpsTime);
            cJSON_AddNumberToObject(root,"speed",jt808._speed);
            cJSON_AddNumberToObject(root,"direction",jt808._direction);
            cJSON_AddNumberToObject(root,"acc",jt808._acc);
            
            char *out;
            
            out=cJSON_Print(root);
            cJSON_Delete(root);
            
            bufferevent_write(g_bev,out,strlen(out));
            
            free(out);

        }
        
    }
    
    printf("sending thread stopped...\n");
    
    return ((void *)0);
    
}

void error_handler(OCI_Error *err)
{
    int err_type = OCI_ErrorGetType(err);
    
    const char *err_msg = OCI_ErrorGetString(err);
    
    printf("** %s - %s\n", err_type == OCI_ERR_WARNING ? "Warning" : "Error", err_msg);
}
