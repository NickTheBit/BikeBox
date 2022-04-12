/* File: recorder.hh
 * Author: Nick Gkloumpos
*/

static const char* TAG_Recorder = "Recorder";

extern "C" void recorderTask(void *);

uint8_t initializeSensors(void);