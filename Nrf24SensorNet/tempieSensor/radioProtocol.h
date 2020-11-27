#ifndef _RADIO_PROTOCOL_H_
#define _RADIO_PROTOCOL_H_

#define HEADER_DATA         1
#define HEADER_COMMAND      2
#define HEADER_REPLY        3

#define CMD_STAY_AWAKE              1  // + on/off
#define CMD_REBOOT                  2
#define CMD_RESET_EEPROM            3
#define CMD_GET_PAYLOAD_ITEM_COUNT  4
#define CMD_GET_PAYLOAD_ITEM_TYPE   5
#define CMD_GET_PARAMETER_COUNT     6
#define CMD_GET_PARAM_NAME          7 // + paramId
#define CMD_RELOAD_EEPROM           8
//#define CMD_SET_PARAM_NAME          8 // + paramId + paramName
#define CMD_GET_NODE_NAME           9
#define CMD_SET_NODE_NAME           10 // + [] + nodeName
#define CMD_GET_PARAM_VALUE         11 // + paramId
#define CMD_SET_PARAM_VALUE         12 // + paramId + paramValue

#define REPLY_INVALID 0xFF

#define PAYLOAD_TYPE_UINT8    1
#define PAYLOAD_TYPE_UINT16   2
#define PAYLOAD_TYPE_UINT32   3
#define PAYLOAD_TYPE_FLOAT    4

struct commandReplyPayload_t {
  uint8_t id;
  uint8_t param;
  uint8_t data[8]; // 8-char string without null termination
};

#endif // _RADIO_PROTOCOL_H_
