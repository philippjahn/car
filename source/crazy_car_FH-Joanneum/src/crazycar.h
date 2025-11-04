
/*************************************************************************
 * Start Stop Tasten
 *************************************************************************
 * auf folgenden PINs befinden sich die Start und die Stop Taste
 *
 * 13 ... Stop
 * 12 ... Start
 *
 * Anmerkung: die Tasten verwenden negative logik
 *************************************************************************/
#define STARTBUTTON 12 //schwarze Taste
#define STOPBUTTON 13  //rote Taste

#define FORWARD_MAX_SPEED_THRESHOLD 150
#define FORWARD_MIN_THRESHOLD 40
#define BACKWARD_THRESHOLD 22

typedef enum
{
    EMERGENCY_STOP,
    STOP,
    DRIVE_FORWARD,
    DRIVE_BACKWARD,
    LEFT,
    RIGHT,
} CAR_STATES;
