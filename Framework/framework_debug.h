#ifndef FRAMEWORK_DEBUG_H
#define FRAMEWORK_DEBUG_H

//���Կ���
#define _FW_DEBUG


#ifdef _FW_DEBUG
	#define fw_printf(...) printf(__VA_ARGS__)
#else
	#define fw_printf(...)  
#endif

#define fw_printfln(format, ...) fw_printf(format"\r\n", ##__VA_ARGS__)

#define fw_Error_Handler() {fw_printf("%s(%d): ", __FILE__, __LINE__);Error_Handler();}

#define fw_Warning() {fw_printf("%s(%d): \r\n", __FILE__, __LINE__);}

#endif
