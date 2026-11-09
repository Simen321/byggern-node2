#pragma once

// CAN message ids shared between node 1 (atmega162) and node 2 (arduino due).

// 0x1xx: node 1 -> node 2
// 0x2xx: node 2 -> node 1   (node 1 only accepts 0x200-0x2FF in its filters)

// node 1 -> node 2
#define CAN_ID_SOLENOID     0x100   // no data. fire the solenoid once
#define CAN_ID_JOYSTICK     0x101   // [x, y, btn]
#define CAN_ID_SLIDER       0x102   // [slider x, touchpad x, touchpad y]
#define CAN_ID_BUTTONS      0x103   // [R2]
#define CAN_ID_GAME_START   0x110   // no data. start watching the ir beam for a lost ball
#define CAN_ID_GAME_STOP    0x111   // no data. stop watching the ir beam (game exited)

// node 2 -> node 1
#define CAN_ID_GAME_OVER    0x200   // no data. the ball went through the ir beam during a game
#define CAN_ID_LOG          0x201   // up to 8 chars of text for the console. '\n' ends a line
