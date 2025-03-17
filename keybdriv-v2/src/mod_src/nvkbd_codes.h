/* SPDX-License-Identifier: GPL-2.0 */
#ifndef NVKBD_CODES_H_
#define NVKBD_CODES_H_

#define NUM_KEYCODES	256
#define MODIFIER_SCANCODE 32
#define NUM_LAYER_KEYCODES 27
#define CONTROL_OFFSET 6

#define CONTROL_START_SCANCODE 28
#define CONTROL_END_SCANCODE 33

#define LAYER_OFFSET(n) ((n) * NUM_LAYER_KEYCODES + (((n) > 0) * CONTROL_OFFSET))

static unsigned int keycodes[NUM_KEYCODES] = {
	// default layer
	[ 1] = KEY_Q,
	[ 2] = KEY_W,
	[ 3] = KEY_E,
	[ 4] = KEY_R,
	[ 5] = KEY_T,
	[ 6] = KEY_Y,
	[ 7] = KEY_U,
	[ 8] = KEY_I,
	[ 9] = KEY_O,

	[10] = KEY_A,
	[11] = KEY_S,
	[12] = KEY_D,
	[13] = KEY_F,
	[14] = KEY_G,
	[15] = KEY_H,
	[16] = KEY_J,
	[17] = KEY_K,
	[18] = KEY_L,

	[19] = KEY_Z,
	[20] = KEY_X,
	[21] = KEY_C,
	[22] = KEY_V,
	[23] = KEY_B,
	[24] = KEY_N,
	[25] = KEY_M,
	[26] = KEY_P,
	[27] = KEY_ENTER,

	// control row
	[28] = KEY_LEFTSHIFT,
	[29] = KEY_LEFTCTRL,
	[30] = KEY_LEFTMETA,
	[31] = KEY_SPACE,
	//[32] = KEY_RIGHTCTRL, //use as internal modeswitch
	[33] = KEY_BACKSPACE,

	// layer 1
	[34] = KEY_ESC,
	[35] = KEY_UP,
	[36] = KEY_BACKSLASH,
	[37] = KEY_1,
	[38] = KEY_2,
	[39] = KEY_3,
	[40] = KEY_LEFTBRACE,
	[41] = KEY_RIGHTBRACE,
	[42] = KEY_SEMICOLON,

	[43] = KEY_LEFT,
	[44] = KEY_DOWN,
	[45] = KEY_RIGHT,
	[46] = KEY_4,
	[47] = KEY_5,
	[48] = KEY_6,
	[49] = KEY_MINUS,
	[50] = KEY_EQUAL,
	[51] = KEY_APOSTROPHE,

	[52] = KEY_TAB,
	[53] = KEY_GRAVE,
	[54] = KEY_0,
	[55] = KEY_7,
	[56] = KEY_8,
	[57] = KEY_9,
	[58] = KEY_COMMA,
	[59] = KEY_DOT,
	[60] = KEY_SLASH,

	/*
	[ 1] = KEY_ESC,
	[ 2] = KEY_1,
	[ 3] = KEY_2,
	[ 4] = KEY_3,
	[ 5] = KEY_4,
	[ 6] = KEY_5,
	[ 7] = KEY_6,
	[ 8] = KEY_7,
	[ 9] = KEY_8,
	[10] = KEY_9,
	[11] = KEY_0,
	[12] = KEY_MINUS,
	[13] = KEY_EQUAL,
	[14] = KEY_BACKSPACE,

	[15] = KEY_TAB,
	[16] = KEY_Q,
	[17] = KEY_W,
	[18] = KEY_E,
	[19] = KEY_R,
	[20] = KEY_T,
	[21] = KEY_Y,
	[22] = KEY_U,
	[23] = KEY_I,
	[24] = KEY_O,
	[25] = KEY_P,
	[26] = KEY_LEFTBRACE,
	[27] = KEY_RIGHTBRACE,
	[28] = KEY_BACKSLASH,  		// figure out what key code the pipe key has

	[29] = KEY_CAPSLOCK,
	[30] = KEY_A,
	[31] = KEY_S,
	[32] = KEY_D,
	[33] = KEY_F,
	[34] = KEY_G,
	[35] = KEY_H,
	[36] = KEY_J,
	[37] = KEY_K,
	[38] = KEY_L,
	[39] = KEY_SEMICOLON,
	[40] = KEY_APOSTROPHE,
	[41] = KEY_ENTER,

	[42] = KEY_LEFTSHIFT,
	[43] = KEY_Z,
	[44] = KEY_X,
	[45] = KEY_C,
	[46] = KEY_V,
	[47] = KEY_B,
	[48] = KEY_N,
	[49] = KEY_M,
	[50] = KEY_COMMA,
	[51] = KEY_DOT,
	[52] = KEY_SLASH,
	[53] = KEY_RIGHTSHIFT,

	[54] = KEY_LEFTCTRL,
	[55] = KEY_LEFTMETA, // left fn key
	[56] = KEY_LEFTALT,
	[57] = KEY_SPACE,
	[58] = KEY_RIGHTALT,
	[59] = KEY_RIGHTCTRL,

	[60] = KEY_UP,
	[61] = KEY_DOWN,
	[62] = KEY_LEFT,
	[63] = KEY_RIGHT
	*/
};


#endif
