#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

//
/*
*@brief							return the signal that the rotation is done or not
*
*@retval	rotation_done		signal that the rotation is done (TRUE/FALSE)
*
*/
uint8_t rotation_finished(void);

//start the PI regulator thread
void pi_regulator_start(void);

#endif /* PI_REGULATOR_H */
