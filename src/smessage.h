#define TAG_BLE_SHORT      1
#define TAG_TIME           2
#define TAG_GPS_POS_LONG   0x1c
#define TAG_BATTERY        4
#define TAG_BLE_LONG       5
#define TAG_LORA_HF        6
#define TAG_SOC            7
#define TAG_PANIC          8
#define TAG_PRESSURE       9



#define HPA_MIN 800
#define HPA_MAX 1200
int smessage_add_PRESSURE_HPA(unsigned char *msg, float hpa)
{
	unsigned short val;

	val = (hpa-HPA_MIN)*65000.0/(HPA_MAX-HPA_MIN);
  msg[0]=TAG_PRESSURE;
	msg[1] = val&0xff;
	msg[2] = (val>>8)*0xff;
	return 3;

}
int smessage_add_PANIC (unsigned char *msg)
{
	msg[0]=TAG_PANIC;
	return 1;
}

int smessage_add_SOC (unsigned char *msg, uint8_t soc)
{
	msg[0]=TAG_SOC;
	msg[1]=soc;
	return 2;
}

int smessage_add_LORA_HF (unsigned char * msg, short id, short dist)
{
	msg[0]= TAG_LORA_HF;
	//memcpy(&msg[1],&id,2);
	msg[1] = id&0xff;
	msg[2]= (id >>8)&0xff;
	memcpy(&msg[1+2],&dist,2);
	return 5;
}
int smessage_add_bat (unsigned char * msg, uint8_t bat)
{
	msg[0]=TAG_BATTERY;
	msg[1]= bat;
	return 2;
}

int smessage_add_GPS (unsigned char * msg, float lon, float lat, float z, short dop)
{
	msg[0]= TAG_GPS_POS_LONG;
	memcpy(&msg[1],&lon,4);
	memcpy(&msg[1+4],&lat,4);
	memcpy(&msg[1+8],&z,4);
	memcpy(&msg[1+12],&dop,2);
	return 15;
}
int smessage_add_ble_short(unsigned char * msg, short ble_id, char rssi)
{
	msg[0]= TAG_BLE_SHORT;
	msg[1] = ble_id&0xff;
	msg[2] = ble_id>>8;
	msg[3] = rssi;
	return 4;
}
int smessage_add_time(char * msg, char hr, char min, char sec);
int smessage_add_bat (unsigned char * msg, uint8_t bat);
//int smessage_add_GPS();



int smessage_parse_ble_short(char *msg, short *id, unsigned char *rssi);


int smessage_parse_time(char * msg, char hr, char min, char sec);
//int smessage_parse_GPS(char *msg, char *id, unsigned char *rssi);

int smessage_parse_ble_short(unsigned char *msg, short *id, unsigned char *rssi)
{
	*id = msg[1] |  msg[2]<<16;
	*rssi = msg[3];
	return 4;
}
#define GATEWAY_ID 12

char * ParseMessage(unsigned short uid, unsigned char * msg, int len)
{
	char tag;
	short ble_id[3];
	unsigned char ble_rss[3];
	uint8_t bles=0;
	int tag_len;
	static char json[100];
        char temp[40];
        int x;
	while (len>0)
	{
	tag = *msg;
	switch (tag)
		{
		case TAG_BLE_SHORT:
			tag_len=smessage_parse_ble_short(msg, &ble_id[bles], &ble_rss[bles]);
			bles++;
		    break;
		}
	msg+=tag_len; len-=tag_len;
	}

  sprintf(json,"{\"tagid\":\"%d\",",uid);
  sprintf(temp,"\"gw_id\":\"%d\",",GATEWAY_ID);
  strcat(json,temp);

  strcat(json,"\"ble\":[");
  for (x=0;x<bles;x++)
  {
	  sprintf(temp,"\"%d\"",ble_id[x]);
	  strcat(json,temp);
	  if (x!=bles-1)
		  strcat(json,",");
  }
  strcat(json,"],");

  strcat(json,"\"ble_rss\":[");
    for (x=0;x<bles;x++)
    {
  	  sprintf(temp,"%d",ble_rss[x]);
  	  strcat(json,temp);
	  if (x!=bles-1)
		  strcat(json,",");
    }
    strcat(json,"]}");

    printf("Json = %s\n",json);
    return json;
}


int BuildTestMessage(unsigned char *msg)
{
	static char pos=0;
	int len=0;
  int tlen;

  pos = pos+1;
if (pos> 80)
  pos=1;
   tlen=smessage_add_ble_short(msg,1,pos);
	msg+=tlen;len+=tlen;

	tlen=smessage_add_ble_short(msg,2,100-pos);
	msg+=tlen;len+=tlen;

	tlen=smessage_add_ble_short(msg,3,100-pos);
	msg+=tlen;len+=tlen;


	return len;


}



char * do_jtest()
{
	unsigned char msg[100];
	int len;
	char *send;

	len = BuildTestMessage(msg);

	send=ParseMessage(33, msg, len);
        return send;
}
