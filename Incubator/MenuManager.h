/*
 * MenuManager.h
 *
 * Created: 3/22/2015 5:52:34 PM
 *  Author: marius
 */ 


#ifndef MENUMANAGER_H_
#define MENUMANAGER_H_

#include <stdbool.h>

#define MENU_BTN_DDR	DDRD
#define MENU_BTN_PORT	PORTD
#define MENU_BTN_PIN	PIND
#define MENU_BTN		PORTD2

#define UP_BTN_DDR	DDRD
#define UP_BTN_PORT	PORTD
#define UP_BTN_PIN	PIND
#define UP_BTN		PORTD0

#define DOWN_BTN_DDR	DDRD
#define DOWN_BTN_PORT	PORTD
#define DOWN_BTN_PIN	PIND
#define DOWN_BTN		PORTD1


volatile bool menuActivated;

void initMenuManager();
void menuLoop();


#endif /* MENUMANAGER_H_ */