#ifndef PID_H
#define PID_H

uint8_t pid_controller(int16_t* const referenceAngle,
                 int16_t* const currentAngle,
                 uint8_t* const direction,
                 int16_t* const jointErrorIntegral,
				 uint8_t switchDirection,
                 const float Kp,
                 const float Ki);

#endif
