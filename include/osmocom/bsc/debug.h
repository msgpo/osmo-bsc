#pragma once

#include <stdio.h>
#include <osmocom/core/linuxlist.h>

#define DEBUG
#include <osmocom/core/logging.h>

#define LOG_ADD_NEWLINE(fmt) ((!fmt || !*fmt || fmt[strlen(fmt)-1] != '\n') ? "\n" : "")

/* Debug Areas of the code */
enum {
	DRLL,
	DMM,
	DRR,
	DRSL,
	DNM,
	DPAG,
	DMEAS,
	DMSC,
	DHO,
	DHODEC,
	DREF,
	DNAT,
	DCTRL,
	DFILTER,
	DPCU,
	DLCLS,
	Debug_LastEntry,
};
