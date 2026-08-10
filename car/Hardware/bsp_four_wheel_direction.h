/**
 ******************************************************************************
 * @file    bsp_four_wheel_direction.h
 * @brief   Physical-forward signs shared by four-wheel test and line mission.
 *
 * The signs are hardware calibration data, not controller signs. A positive
 * logical wheel command means vehicle-forward only after multiplication by
 * the corresponding sign below.
 *
 * Evidence: FourWheelTb6612Debug physical-forward qualification.
 ******************************************************************************
 */

#ifndef __BSP_FOUR_WHEEL_DIRECTION_H
#define __BSP_FOUR_WHEEL_DIRECTION_H

/* Front TB6612: PA2/PA3, PE2--PE6. */
#define FRONT_LEFT_FORWARD_SIGN       (-1)
#define FRONT_RIGHT_FORWARD_SIGN      (-1)

/* Rear TB6612: PE13/PE14, PF1--PF4, PB9. */
#define REAR_LEFT_FORWARD_SIGN        (+1)
#define REAR_RIGHT_FORWARD_SIGN       (-1)

#endif /* __BSP_FOUR_WHEEL_DIRECTION_H */
