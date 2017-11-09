//
//  gps_hub.h
//  GPSHub
//
//  Created by road on 7/11/16.
//  Copyright (c) 2016 road. All rights reserved.
//

#ifndef GPSHub_gps_hub_h
#define GPSHub_gps_hub_h

#include <ocilib.h>

#define GET_ARRAY_LEN(array,len) {len = (sizeof(array) / sizeof(array[0]));}

struct st_lsnr_context{
    
    OCI_Pool * pool;
    struct event_base * base;
    
};

struct st_read_context{
    
    struct st_lsnr_context * ctx;
    struct evbuffer * buf;
    int isSendingSocket;
};


typedef struct JT808{
    
    int _messageType;
    short _messageBodyProperty;
    unsigned char _sim[13];
    float _lng;
    float _lat;
    short _speed;
    short _direction;
    short _high;
    unsigned char _gpsTime[13];
    int _acc;
    int _len;
    
} JT808Type;

#endif
