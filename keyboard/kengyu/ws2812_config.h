/*
 * light_ws2812_config.h
 *
 * Created: 18.01.2014 09:58:15
 *
 * User Configuration file for the light_ws2812_lib
 *
 */ 


#ifndef WS2812_CONFIG_H_
#define WS2812_CONFIG_H_

///////////////////////////////////////////////////////////////////////
// Define I/O pin
///////////////////////////////////////////////////////////////////////
#ifdef KENG
#define ws2812_port B     // Data port 
#define ws2812_pin  0     // Data out pin
#else
#define ws2812_port C     // Data port 
#define ws2812_pin  6     // Data out pin
#endif

#endif /* WS2812_CONFIG_H_ */
